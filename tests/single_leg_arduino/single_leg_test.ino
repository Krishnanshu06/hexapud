/*
 * HexaPud - Single Leg Test Sketch (Arduino C++)
 * ESP32 + PCA9685 + 3-DOF Leg (Coxa/Femur/Tibia)
 *
 * Dimensions: Coxa = 51mm, Femur = 80mm, Tibia = 128mm
 * Serial Baud: 115200
 */

#include <Wire.h>
#include <Adafruit_PWMServoDriver.h>
#include <math.h>

// --- Hardware Configuration ---
// ─────────────────────────────────────────────

// I2C pins for PCA9685 (SDA=21, SCL=23)
#define I2C_SDA 21
#define I2C_SCL 23

// PCA9685 I2C address (default 0x40, check if you've soldered address jumpers)
#define PCA_ADDRESS 0x40

// Which PCA9685 channels are your 3 servos connected to?
#define COXA_CHANNEL   1    // Channel for coxa (hip) servo
#define FEMUR_CHANNEL  2    // Channel for femur (thigh) servo
#define TIBIA_CHANNEL  3    // Channel for tibia (shin) servo

// ─────────────────────────────────────────────
// ██  SERVO CALIBRATION — TUNE THESE FOR YOUR SERVOS
// ─────────────────────────────────────────────

// Pulse width range in microseconds
// Typical: 500-2500 for most servos. Adjust if your servos behave oddly.
#define SERVO_MIN_US  500
#define SERVO_MAX_US  2500

// Servo angle limits (degrees) — found by testing each servo
// Start wide (10-170) and narrow down if the servo buzzes at the extremes
#define COXA_MIN_ANGLE   10.0f
#define COXA_MAX_ANGLE   170.0f
#define FEMUR_MIN_ANGLE  10.0f
#define FEMUR_MAX_ANGLE  170.0f
#define TIBIA_MIN_ANGLE  10.0f
#define TIBIA_MAX_ANGLE  170.0f

// Servo direction: set to -1 if a servo moves the OPPOSITE way than expected
// How to test: type "sweep 0" — if the coxa swings the wrong way, flip to -1
#define COXA_DIRECTION   1    // 1 = normal, -1 = reversed
#define FEMUR_DIRECTION  1
#define TIBIA_DIRECTION  1

// Servo center offsets: if 90° doesn't EXACTLY center the joint, adjust here
// Positive = shifts the servo clockwise, Negative = counter-clockwise
// Tip: use "center" command, see if leg is perfectly neutral. If not, tweak.
#define COXA_OFFSET    0.0f
#define FEMUR_OFFSET   0.0f
#define TIBIA_OFFSET   0.0f

// ─────────────────────────────────────────────
// ██  LEG DIMENSIONS (mm)
// ─────────────────────────────────────────────

#define COXA_LEN    51.0f    // L1: coxa joint center → femur joint center
#define FEMUR_LEN   80.0f    // L2: femur joint center → tibia joint center
#define TIBIA_LEN  128.0f    // L3: tibia joint center → foot tip

// ─────────────────────────────────────────────
// ██  MOVEMENT PARAMETERS
// ─────────────────────────────────────────────

// Standing position: where the foot rests relative to the coxa joint (mm)
// X = outward from body, Y = sideways, Z = vertical (negative = below coxa)
//
//   Your neutral (all servos 90°):
//     foot at X = 51+80 = 131mm out, Z = -128mm below (tibia length down)
//
//   A good standing pose bends the knee a bit for stability:
#define STANDING_X     130.0f    // mm outward from coxa joint
#define STANDING_Z    -140.0f    // mm below coxa joint (negative = down)

#define STEP_HEIGHT        40.0f   // How high the foot lifts during walk (mm)
#define STRIDE_LENGTH      50.0f   // How far forward/back each step goes (mm)
#define MAX_DEG_PER_TICK    3.0f   // Rate limit: max degrees per update cycle
#define UPDATE_INTERVAL_MS 20      // Main loop tick rate (20ms = 50Hz)

// ═══════════════════════════════════════════════════════════════
//  INTERNALS — You shouldn't need to change anything below
// ═══════════════════════════════════════════════════════════════

Adafruit_PWMServoDriver pwm = Adafruit_PWMServoDriver(PCA_ADDRESS);

// Current actual servo angles (for rate limiting)
float currentAngles[3] = {90.0f, 90.0f, 90.0f};

// IK result — these are GEOMETRIC angles, not servo angles
struct IKResult {
    float coxa;    // Rotation from straight-out, degrees. 0° = straight out.
    float femur;   // Angle below horizontal, degrees. 0° = horizontal.
    float tibia;   // Angle between femur and tibia, degrees. 90° = perpendicular.
    bool valid;
};

// Demo state
enum DemoMode { DEMO_NONE, DEMO_WAVE, DEMO_WALK, DEMO_SWEEP };
DemoMode currentDemo = DEMO_NONE;
int sweepJoint = 0;
float demoPhase = 0.0f;
unsigned long lastUpdate = 0;

// ─────────────────────────────────────────────
//  Servo output
// ─────────────────────────────────────────────

void setServoPWM(int channel, float angleDeg) {
    // Convert angle to pulse width in microseconds
    float pulseUs = SERVO_MIN_US + (angleDeg / 180.0f) * (SERVO_MAX_US - SERVO_MIN_US);
    
    // PCA9685: 4096 ticks per cycle, 50Hz = 20000μs per cycle
    int pulseTicks = (int)(pulseUs / 20000.0f * 4096.0f);
    pulseTicks = constrain(pulseTicks, 0, 4095);
    
    pwm.setPWM(channel, 0, pulseTicks);
}

// ─────────────────────────────────────────────
//  IK angle → Servo angle conversion
// ─────────────────────────────────────────────
//
//  YOUR MOUNTING CONVENTION (all servos at 90° = neutral):
//
//    Coxa servo 90°  = leg straight out      → IK coxa 0°  = servo 90°
//    Femur servo 90° = femur horizontal      → IK femur 0° = servo 90°
//    Tibia servo 90° = tibia perpendicular   → IK tibia 90° = servo 90°
//
//  So the mapping is:
//    servo_coxa  = 90 + IK_coxa
//    servo_femur = 90 + IK_femur  (+ means femur tilts downward)
//    servo_tibia = IK_tibia       (direct mapping)
//

float ikToServoCoxa(float ikAngle) {
    return 90.0f + ikAngle;
}

float ikToServoFemur(float ikAngle) {
    return 90.0f + ikAngle;
}

float ikToServoTibia(float ikAngle) {
    return ikAngle;
}

// Apply direction, offset, clamping, rate limiting, then output to PCA9685
void applyServoAngle(int jointIndex, float servoAngle) {
    int channel;
    float minA, maxA, offset;
    int dir;
    
    switch (jointIndex) {
        case 0: channel = COXA_CHANNEL;  minA = COXA_MIN_ANGLE;  maxA = COXA_MAX_ANGLE;  offset = COXA_OFFSET;  dir = COXA_DIRECTION;  break;
        case 1: channel = FEMUR_CHANNEL; minA = FEMUR_MIN_ANGLE; maxA = FEMUR_MAX_ANGLE; offset = FEMUR_OFFSET; dir = FEMUR_DIRECTION; break;
        case 2: channel = TIBIA_CHANNEL; minA = TIBIA_MIN_ANGLE; maxA = TIBIA_MAX_ANGLE; offset = TIBIA_OFFSET; dir = TIBIA_DIRECTION; break;
        default: return;
    }
    
    // Apply direction flip
    float angle = (dir == -1) ? (180.0f - servoAngle) : servoAngle;
    
    // Apply calibration offset
    angle += offset;
    
    // Clamp to safe servo limits
    angle = constrain(angle, minA, maxA);
    
    // Rate limiting: don't jump more than MAX_DEG_PER_TICK per update
    float diff = angle - currentAngles[jointIndex];
    if (diff > MAX_DEG_PER_TICK)  diff = MAX_DEG_PER_TICK;
    if (diff < -MAX_DEG_PER_TICK) diff = -MAX_DEG_PER_TICK;
    angle = currentAngles[jointIndex] + diff;
    
    // Store and output
    currentAngles[jointIndex] = angle;
    setServoPWM(channel, angle);
}

// Set all three servos immediately (bypasses rate limiting)
void setServosImmediate(float coxa, float femur, float tibia) {
    // Apply direction and offset
    float c = (COXA_DIRECTION == -1 ? 180.0f - coxa : coxa) + COXA_OFFSET;
    float f = (FEMUR_DIRECTION == -1 ? 180.0f - femur : femur) + FEMUR_OFFSET;
    float t = (TIBIA_DIRECTION == -1 ? 180.0f - tibia : tibia) + TIBIA_OFFSET;
    
    c = constrain(c, COXA_MIN_ANGLE, COXA_MAX_ANGLE);
    f = constrain(f, FEMUR_MIN_ANGLE, FEMUR_MAX_ANGLE);
    t = constrain(t, TIBIA_MIN_ANGLE, TIBIA_MAX_ANGLE);
    
    currentAngles[0] = c;
    currentAngles[1] = f;
    currentAngles[2] = t;
    
    setServoPWM(COXA_CHANNEL, c);
    setServoPWM(FEMUR_CHANNEL, f);
    setServoPWM(TIBIA_CHANNEL, t);
}

// IK → servo conversion → apply all three (with rate limiting)
void applyIKResult(IKResult ik) {
    float sc = ikToServoCoxa(ik.coxa);
    float sf = ikToServoFemur(ik.femur);
    float st = ikToServoTibia(ik.tibia);
    
    applyServoAngle(0, sc);
    applyServoAngle(1, sf);
    applyServoAngle(2, st);
}

// IK → servo conversion → set immediately (no rate limiting)
void applyIKImmediate(IKResult ik) {
    float sc = ikToServoCoxa(ik.coxa);
    float sf = ikToServoFemur(ik.femur);
    float st = ikToServoTibia(ik.tibia);
    
    setServosImmediate(sc, sf, st);
}

// ─────────────────────────────────────────────
//  Inverse Kinematics Solver
// ─────────────────────────────────────────────
//
//  Input:  Desired foot position (x, y, z) relative to coxa joint (mm)
//            x = outward from body (positive = further out)
//            y = sideways (positive = one side, negative = other)
//            z = vertical (negative = below coxa joint)
//
//  Output: IK geometric angles:
//            coxa  = rotation from straight-out (degrees)
//            femur = angle below horizontal (degrees, 0=horizontal, +=down)
//            tibia = angle between femur and tibia segments (degrees, 90=perpendicular)
//

IKResult solveIK(float x, float y, float z) {
    IKResult result;
    result.valid = false;
    
    // Step 1: Coxa angle (top-down rotation)
    // 0° = pointing in +X direction (straight out)
    result.coxa = atan2f(y, x) * 180.0f / M_PI;
    
    // Step 2: Project into the femur-tibia 2D plane
    // dHoriz = horizontal distance from femur joint to foot
    float horizDist = sqrtf(x * x + y * y);
    float dHoriz = horizDist - COXA_LEN;
    
    if (dHoriz < 0) {
        Serial.println("  [IK ERROR] Foot is closer than coxa length! Too close to body.");
        return result;
    }
    
    // D = straight-line distance from femur joint to foot
    float D = sqrtf(dHoriz * dHoriz + z * z);
    
    // Check reachability
    float maxReach = FEMUR_LEN + TIBIA_LEN;
    float minReach = fabsf(FEMUR_LEN - TIBIA_LEN);
    
    if (D > maxReach) {
        Serial.printf("  [IK ERROR] Too far! D=%.1fmm, max reach=%.1fmm\n", D, maxReach);
        return result;
    }
    if (D < minReach) {
        Serial.printf("  [IK ERROR] Too close! D=%.1fmm, min reach=%.1fmm\n", D, minReach);
        return result;
    }
    
    // Step 3: Femur angle
    //   alpha = angle of line from femur joint to foot, below horizontal
    //   beta  = angle between femur segment and line-to-foot, at the femur joint
    //   femur_angle = alpha - beta  (elbow-down configuration = normal walking)
    //
    //   alpha - beta: femur is ABOVE the line to foot (knee bends down) ← we want this
    //   alpha + beta: femur is BELOW the line to foot (knee bends up)   ← weird, don't use
    //
    float alpha = atan2f(-z, dHoriz);   // positive when foot is below horizontal
    
    float cosB = (FEMUR_LEN * FEMUR_LEN + D * D - TIBIA_LEN * TIBIA_LEN) / (2.0f * FEMUR_LEN * D);
    cosB = constrain(cosB, -1.0f, 1.0f);
    float beta = acosf(cosB);
    
    result.femur = (alpha - beta) * 180.0f / M_PI;   // degrees below horizontal
    
    // Step 4: Tibia angle (angle between femur and tibia segments)
    //   When = 180°: fully extended (straight leg)
    //   When = 90°:  right angle at knee
    //   When < 90°:  folded tight
    float cosG = (FEMUR_LEN * FEMUR_LEN + TIBIA_LEN * TIBIA_LEN - D * D) / (2.0f * FEMUR_LEN * TIBIA_LEN);
    cosG = constrain(cosG, -1.0f, 1.0f);
    result.tibia = acosf(cosG) * 180.0f / M_PI;
    
    result.valid = true;
    return result;
}

// ─────────────────────────────────────────────
//  Preset Positions
// ─────────────────────────────────────────────

void moveToStand() {
    Serial.println("Moving to standing position...");
    Serial.printf("  Target foot: (%.0f, 0, %.0f) mm\n", STANDING_X, STANDING_Z);
    
    IKResult ik = solveIK(STANDING_X, 0, STANDING_Z);
    if (ik.valid) {
        float sc = ikToServoCoxa(ik.coxa);
        float sf = ikToServoFemur(ik.femur);
        float st = ikToServoTibia(ik.tibia);
        Serial.printf("  IK geometric:  coxa=%.1f°  femur=%.1f°  tibia=%.1f°\n", ik.coxa, ik.femur, ik.tibia);
        Serial.printf("  Servo outputs: coxa=%.1f°  femur=%.1f°  tibia=%.1f°\n", sc, sf, st);
        applyIKImmediate(ik);
    } else {
        Serial.println("  Standing position unreachable! Adjust STANDING_X and STANDING_Z.");
    }
}

void moveToNeutral() {
    Serial.println("Moving to neutral (all servos 90°)...");
    Serial.println("  Coxa: straight out, Femur: horizontal, Tibia: 90° down");
    setServosImmediate(90.0f, 90.0f, 90.0f);
}

void moveToLift() {
    Serial.println("Lifting foot from standing...");
    float liftZ = STANDING_Z + STEP_HEIGHT;
    Serial.printf("  Target foot: (%.0f, 0, %.0f) mm\n", STANDING_X, liftZ);
    
    IKResult ik = solveIK(STANDING_X, 0, liftZ);
    if (ik.valid) {
        float sc = ikToServoCoxa(ik.coxa);
        float sf = ikToServoFemur(ik.femur);
        float st = ikToServoTibia(ik.tibia);
        Serial.printf("  IK geometric:  coxa=%.1f°  femur=%.1f°  tibia=%.1f°\n", ik.coxa, ik.femur, ik.tibia);
        Serial.printf("  Servo outputs: coxa=%.1f°  femur=%.1f°  tibia=%.1f°\n", sc, sf, st);
        applyIKImmediate(ik);
    } else {
        Serial.println("  Lift position unreachable!");
    }
}

// ─────────────────────────────────────────────
//  Demo Animations (called from main loop)
// ─────────────────────────────────────────────

void updateWaveDemo() {
    demoPhase += 0.02f;
    if (demoPhase > 2.0f * M_PI) demoPhase -= 2.0f * M_PI;
    
    // Wave the leg: foot traces a circle in the Y-Z plane
    float x = STANDING_X;
    float y = 40.0f * sinf(demoPhase);
    float z = STANDING_Z + STEP_HEIGHT + 25.0f * cosf(demoPhase);
    
    IKResult ik = solveIK(x, y, z);
    if (ik.valid) {
        applyIKResult(ik);
    }
}

void updateWalkDemo() {
    demoPhase += 0.015f;
    if (demoPhase >= 1.0f) demoPhase -= 1.0f;
    
    float x, z;
    
    if (demoPhase < 0.5f) {
        // Swing phase: lift and move forward
        float t = demoPhase / 0.5f;
        x = STANDING_X + STRIDE_LENGTH * (-1.0f + 2.0f * t);
        z = STANDING_Z + STEP_HEIGHT * sinf(M_PI * t);
    } else {
        // Stance phase: on ground, move backward
        float t = (demoPhase - 0.5f) / 0.5f;
        x = STANDING_X + STRIDE_LENGTH * (1.0f - 2.0f * t);
        z = STANDING_Z;
    }
    
    IKResult ik = solveIK(x, 0, z);
    if (ik.valid) {
        applyIKResult(ik);
    }
}

void updateSweepDemo() {
    demoPhase += 0.005f;
    if (demoPhase > 2.0f * M_PI) demoPhase -= 2.0f * M_PI;
    
    // Sweep selected joint from 45° to 135° (centered on 90°)
    float angle = 90.0f + 45.0f * sinf(demoPhase);
    applyServoAngle(sweepJoint, angle);
    
    // Print position every ~1 second
    static unsigned long lastPrint = 0;
    if (millis() - lastPrint > 1000) {
        const char* names[] = {"Coxa", "Femur", "Tibia"};
        Serial.printf("  Sweeping %s: servo=%.1f°\n", names[sweepJoint], angle);
        lastPrint = millis();
    }
}

// ─────────────────────────────────────────────
//  Serial Command Parser
// ─────────────────────────────────────────────

String inputBuffer = "";

void printHelp() {
    Serial.println();
    Serial.println("╔══════════════════════════════════════════════════════════╗");
    Serial.println("║          HexaPud Single Leg Test v2 — Commands          ║");
    Serial.println("╠══════════════════════════════════════════════════════════╣");
    Serial.println("║ angles <coxa> <femur> <tibia>  Set raw servo angles (°) ║");
    Serial.println("║ foot <x> <y> <z>              Set foot pos via IK (mm)  ║");
    Serial.println("║ stand                         Bent-knee standing pose   ║");
    Serial.println("║ neutral                       All servos to 90° (L pose)║");
    Serial.println("║ lift                          Lift foot up from stand   ║");
    Serial.println("║ center                        Same as neutral           ║");
    Serial.println("║ wave                          Wave demo loop            ║");
    Serial.println("║ walk                          Step cycle demo           ║");
    Serial.println("║ sweep <0|1|2>                 Sweep one joint           ║");
    Serial.println("║ stop                          Stop demo                 ║");
    Serial.println("║ help                          Show this menu            ║");
    Serial.println("╚══════════════════════════════════════════════════════════╝");
    Serial.println();
    Serial.printf("Leg dims: Coxa=%.0fmm  Femur=%.0fmm  Tibia=%.0fmm\n", COXA_LEN, FEMUR_LEN, TIBIA_LEN);
    Serial.printf("Reach: max=%.0fmm  min=%.0fmm  (from femur joint)\n", FEMUR_LEN + TIBIA_LEN, fabsf(FEMUR_LEN - TIBIA_LEN));
    Serial.printf("Standing: X=%.0fmm  Z=%.0fmm\n", STANDING_X, STANDING_Z);
    Serial.println();
    Serial.println("Coordinate system (relative to coxa joint):");
    Serial.println("  +X = outward from body    +Y = sideways");
    Serial.println("  -Z = below coxa (down)    +Z = above coxa (up)");
    Serial.println();
}

void processCommand(String cmd) {
    cmd.trim();
    if (cmd.length() == 0) return;
    
    Serial.printf("\n> %s\n", cmd.c_str());
    
    if (cmd == "help") {
        printHelp();
    }
    else if (cmd == "stand") {
        currentDemo = DEMO_NONE;
        moveToStand();
    }
    else if (cmd == "neutral" || cmd == "center") {
        currentDemo = DEMO_NONE;
        moveToNeutral();
    }
    else if (cmd == "lift") {
        currentDemo = DEMO_NONE;
        moveToLift();
    }
    else if (cmd == "wave") {
        Serial.println("Starting wave demo... (type 'stop' to end)");
        currentDemo = DEMO_WAVE;
        demoPhase = 0;
    }
    else if (cmd == "walk") {
        Serial.println("Starting walk cycle demo... (type 'stop' to end)");
        currentDemo = DEMO_WALK;
        demoPhase = 0;
    }
    else if (cmd.startsWith("sweep")) {
        int joint = 0;
        if (cmd.length() > 6) {
            joint = cmd.substring(6).toInt();
        }
        joint = constrain(joint, 0, 2);
        const char* names[] = {"Coxa", "Femur", "Tibia"};
        Serial.printf("Sweeping %s (joint %d) from 45° to 135°... (type 'stop' to end)\n", names[joint], joint);
        currentDemo = DEMO_SWEEP;
        sweepJoint = joint;
        demoPhase = 0;
    }
    else if (cmd == "stop") {
        currentDemo = DEMO_NONE;
        Serial.println("Demo stopped.");
    }
    else if (cmd.startsWith("angles")) {
        currentDemo = DEMO_NONE;
        float c, f, t;
        int n = sscanf(cmd.c_str(), "angles %f %f %f", &c, &f, &t);
        if (n == 3) {
            Serial.printf("  Setting raw servo angles: coxa=%.1f°  femur=%.1f°  tibia=%.1f°\n", c, f, t);
            setServosImmediate(c, f, t);
        } else {
            Serial.println("  Usage: angles <coxa> <femur> <tibia>");
            Serial.println("  Example: angles 90 90 90");
        }
    }
    else if (cmd.startsWith("foot")) {
        currentDemo = DEMO_NONE;
        float x, y, z;
        int n = sscanf(cmd.c_str(), "foot %f %f %f", &x, &y, &z);
        if (n == 3) {
            Serial.printf("  Requested foot position: (%.1f, %.1f, %.1f) mm\n", x, y, z);
            
            IKResult ik = solveIK(x, y, z);
            if (ik.valid) {
                float sc = ikToServoCoxa(ik.coxa);
                float sf = ikToServoFemur(ik.femur);
                float st = ikToServoTibia(ik.tibia);
                Serial.printf("  IK geometric:  coxa=%.1f°  femur=%.1f°  tibia=%.1f°\n", ik.coxa, ik.femur, ik.tibia);
                Serial.printf("  Servo outputs: coxa=%.1f°  femur=%.1f°  tibia=%.1f°\n", sc, sf, st);
                applyIKImmediate(ik);
            } else {
                Serial.println("  Position unreachable!");
            }
        } else {
            Serial.println("  Usage: foot <x> <y> <z>");
            Serial.println("  Example: foot 130 0 -140   (standing position)");
            Serial.println("  Example: foot 131 0 -128   (neutral / L-shape)");
        }
    }
    else {
        Serial.printf("  Unknown command: '%s'. Type 'help' for commands.\n", cmd.c_str());
    }
}

// ─────────────────────────────────────────────
//  Setup & Main Loop
// ─────────────────────────────────────────────

void setup() {
    Serial.begin(115200);
    delay(1000);
    
    Serial.println();
    Serial.println("══════════════════════════════════════════════════");
    Serial.println("  HexaPud — Single Leg Test v2");
    Serial.println("══════════════════════════════════════════════════");
    
    // Initialize I2C
    Wire.begin(I2C_SDA, I2C_SCL);
    Serial.printf("I2C: SDA=%d  SCL=%d\n", I2C_SDA, I2C_SCL);
    
    // Initialize PCA9685
    pwm.begin();
    pwm.setPWMFreq(50);  // 50Hz for servos
    delay(10);
    Serial.printf("PCA9685: address=0x%02X  freq=50Hz\n", PCA_ADDRESS);
    
    // Verify neutral position IK
    Serial.println();
    Serial.println("── Verifying IK at neutral pose (all servos 90°) ──");
    Serial.println("   Foot should be at (131, 0, -128):");
    IKResult test = solveIK(COXA_LEN + FEMUR_LEN, 0, -TIBIA_LEN);
    if (test.valid) {
        Serial.printf("   IK geometric:  coxa=%.1f°  femur=%.1f°  tibia=%.1f°\n", test.coxa, test.femur, test.tibia);
        Serial.printf("   Servo outputs: coxa=%.1f°  femur=%.1f°  tibia=%.1f°\n", 
                       ikToServoCoxa(test.coxa), ikToServoFemur(test.femur), ikToServoTibia(test.tibia));
        Serial.println("   (Should be: servo coxa=90, femur=90, tibia=90)");
    }
    Serial.println();
    
    // Start at neutral position (all 90°)
    Serial.println("Setting servos to neutral (90°)...");
    setServosImmediate(90.0f, 90.0f, 90.0f);
    delay(500);
    
    printHelp();
    Serial.println("Ready! Type a command and press Enter.\n");
}

void loop() {
    // Read serial input
    while (Serial.available()) {
        char c = Serial.read();
        if (c == '\n' || c == '\r') {
            if (inputBuffer.length() > 0) {
                processCommand(inputBuffer);
                inputBuffer = "";
            }
        } else {
            inputBuffer += c;
        }
    }
    
    // Run demo animations at fixed rate
    unsigned long now = millis();
    if (now - lastUpdate >= UPDATE_INTERVAL_MS) {
        lastUpdate = now;
        
        switch (currentDemo) {
            case DEMO_WAVE:  updateWaveDemo();  break;
            case DEMO_WALK:  updateWalkDemo();  break;
            case DEMO_SWEEP: updateSweepDemo(); break;
            default: break;
        }
    }
}
