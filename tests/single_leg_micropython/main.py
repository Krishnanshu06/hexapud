"""
HexaPud - Single Leg Diagnostic & IK Test (MicroPython)
ESP32 + PCA9685 (0x40) + 3-DOF Leg (Coxa/Femur/Tibia)
"""

from machine import Pin, I2C
import math
import time
from pca9685 import PCA9685

# --- Hardware Configuration ---
I2C_SDA = 21
I2C_SCL = 23
PCA_ADDRESS = 0x40

COXA_CHANNEL = 2
FEMUR_CHANNEL = 0
TIBIA_CHANNEL = 1

# --- Servo Pulse Settings ---

SERVO_MIN_US = 500
SERVO_MAX_US = 2500

COXA_MIN_ANGLE  = 10.0
COXA_MAX_ANGLE  = 170.0
FEMUR_MIN_ANGLE = 10.0
FEMUR_MAX_ANGLE = 170.0
TIBIA_MIN_ANGLE = 10.0
TIBIA_MAX_ANGLE = 170.0

# Set to -1 if a servo moves the opposite way than expected
COXA_DIRECTION  = 1
FEMUR_DIRECTION = 1
TIBIA_DIRECTION = 1

# Fine-tune offsets (degrees) if 90° isn't perfectly neutral
COXA_OFFSET  = 0.0
FEMUR_OFFSET = 0.0
TIBIA_OFFSET = 0.0

# ─────────────────────────────────────────────
# ██  LEG DIMENSIONS & SMOOTHING PARAMETERS
# ─────────────────────────────────────────────

COXA_LEN  = 51.0
FEMUR_LEN = 80.0
TIBIA_LEN = 128.0

STANDING_X    = 130.0   # mm outward from coxa
STANDING_Z    = -140.0  # mm below coxa
STEP_HEIGHT   = 40.0    # mm foot lift height
STRIDE_LENGTH = 50.0    # mm forward/back sweep

# Smoothness settings
UPDATE_MS         = 10     # 10ms = 100Hz update rate for ultra-smooth steps
MOVE_DURATION_MS  = 600    # Default time (ms) for point-to-point moves (stand, lift, foot)
WALK_CYCLE_SEC    = 1.8    # Seconds for one full walk step cycle (higher = slower & smoother)
SMOOTHING_ALPHA   = 0.25   # Exponential low-pass filter (0.1 = super soft, 1.0 = instant)

# ─────────────────────────────────────────────
#  INITIALIZATION
# ─────────────────────────────────────────────

time.sleep_ms(100)  # Let I2C rail settle on boot
i2c = I2C(0, scl=Pin(I2C_SCL), sda=Pin(I2C_SDA), freq=100000)
devices = i2c.scan()
print("I2C scan on SDA={}, SCL={} found: {}".format(
    I2C_SDA, I2C_SCL, [hex(d) for d in devices]))

if not devices:
    raise RuntimeError(
        "\n[HARDWARE ERROR] No I2C devices found!\n"
        "  1. Check that PCA9685 VCC is connected to ESP32 3.3V\n"
        "  2. Check that PCA9685 GND is connected to ESP32 GND\n"
        "  3. Check SDA -> GPIO 21 and SCL -> GPIO 23\n"
        "  4. Make sure the LED on the PCA9685 board is lit."
    )

if PCA_ADDRESS not in devices:
    non_allcall = [d for d in devices if d != 0x70]
    if non_allcall:
        print("PCA9685 not at 0x{:02X}, using detected address 0x{:02X}".format(
            PCA_ADDRESS, non_allcall[0]))
        PCA_ADDRESS = non_allcall[0]

pwm = PCA9685(i2c, address=PCA_ADDRESS)
pwm.set_pwm_freq(50)

# Track current servo angles (in logical servo space before direction/offset)
current_angles = [90.0, 90.0, 90.0]


def clamp(val, min_val, max_val):
    return max(min_val, min(max_val, val))


def ease_in_out_cosine(t):
    """Smooth S-curve easing: maps t in [0, 1] -> [0, 1] with zero velocity at ends."""
    t = clamp(t, 0.0, 1.0)
    return 0.5 * (1.0 - math.cos(math.pi * t))


# ─────────────────────────────────────────────
#  SERVO OUTPUT & MAPPING
# ─────────────────────────────────────────────

def set_servo_pwm(channel, angle_deg):
    pulse_us = SERVO_MIN_US + (angle_deg / 180.0) * (SERVO_MAX_US - SERVO_MIN_US)
    pwm.set_servo_pulse(channel, pulse_us)


def _logical_to_hw(joint_index, servo_angle):
    """Apply direction, offset, and safety clamp to a logical servo angle."""
    if joint_index == 0:
        ch, min_a, max_a, off, d = COXA_CHANNEL, COXA_MIN_ANGLE, COXA_MAX_ANGLE, COXA_OFFSET, COXA_DIRECTION
    elif joint_index == 1:
        ch, min_a, max_a, off, d = FEMUR_CHANNEL, FEMUR_MIN_ANGLE, FEMUR_MAX_ANGLE, FEMUR_OFFSET, FEMUR_DIRECTION
    else:
        ch, min_a, max_a, off, d = TIBIA_CHANNEL, TIBIA_MIN_ANGLE, TIBIA_MAX_ANGLE, TIBIA_OFFSET, TIBIA_DIRECTION

    hw_angle = (180.0 - servo_angle) if d == -1 else servo_angle
    hw_angle += off
    hw_angle = clamp(hw_angle, min_a, max_a)
    return ch, hw_angle


def write_joint_direct(joint_index, servo_angle):
    """Write logical servo angle directly to hardware and update state."""
    current_angles[joint_index] = servo_angle
    ch, hw_angle = _logical_to_hw(joint_index, servo_angle)
    set_servo_pwm(ch, hw_angle)


def write_joint_filtered(joint_index, target_angle, alpha=SMOOTHING_ALPHA):
    """Exponentially smoothed step toward target_angle for continuous loops."""
    smoothed = current_angles[joint_index] + alpha * (target_angle - current_angles[joint_index])
    write_joint_direct(joint_index, smoothed)


def move_smooth(target_coxa, target_femur, target_tibia, duration_ms=None):
    """
    Smoothly glide all 3 joints from their current angles to target angles
    over duration_ms using a cosine S-curve (zero jerk at start and stop).
    """
    if duration_ms is None:
        duration_ms = MOVE_DURATION_MS

    start_c, start_f, start_t = current_angles[0], current_angles[1], current_angles[2]
    steps = max(1, int(duration_ms // UPDATE_MS))

    for i in range(1, steps + 1):
        s = ease_in_out_cosine(i / steps)
        c = start_c + (target_coxa  - start_c) * s
        f = start_f + (target_femur - start_f) * s
        t = start_t + (target_tibia - start_t) * s

        write_joint_direct(0, c)
        write_joint_direct(1, f)
        write_joint_direct(2, t)
        time.sleep_ms(UPDATE_MS)


def ik_to_servo_coxa(ik_angle):
    return 90.0 + ik_angle


def ik_to_servo_femur(ik_angle):
    return 90.0 + ik_angle


def ik_to_servo_tibia(ik_angle):
    return ik_angle


def apply_ik_result(ik_res, smooth_move=False, duration_ms=None):
    sc = ik_to_servo_coxa(ik_res['coxa'])
    sf = ik_to_servo_femur(ik_res['femur'])
    st = ik_to_servo_tibia(ik_res['tibia'])
    if smooth_move:
        move_smooth(sc, sf, st, duration_ms)
    else:
        write_joint_filtered(0, sc)
        write_joint_filtered(1, sf)
        write_joint_filtered(2, st)


# ─────────────────────────────────────────────
#  INVERSE KINEMATICS SOLVER
# ─────────────────────────────────────────────

def solve_ik(x, y, z):
    """
    Input: desired foot (x, y, z) in mm relative to coxa joint.
    Returns dict: {'coxa': deg, 'femur': deg, 'tibia': deg, 'valid': bool}
    """
    res = {'coxa': 0.0, 'femur': 0.0, 'tibia': 0.0, 'valid': False}

    # Step 1: Coxa angle
    res['coxa'] = math.atan2(y, x) * 180.0 / math.pi

    # Step 2: Project into 2D femur-tibia plane
    horiz_dist = math.sqrt(x * x + y * y)
    d_horiz = horiz_dist - COXA_LEN

    if d_horiz < 0:
        print("  [IK ERROR] Foot closer than coxa length!")
        return res

    D = math.sqrt(d_horiz * d_horiz + z * z)
    max_reach = FEMUR_LEN + TIBIA_LEN
    min_reach = abs(FEMUR_LEN - TIBIA_LEN)

    if D > max_reach or D < min_reach:
        print("  [IK ERROR] Unreachable! D={:.1f}mm (range: {:.1f} to {:.1f}mm)".format(D, min_reach, max_reach))
        return res

    # Step 3: Femur angle (elbow-down: alpha - beta)
    alpha = math.atan2(-z, d_horiz)
    cos_b = (FEMUR_LEN**2 + D**2 - TIBIA_LEN**2) / (2.0 * FEMUR_LEN * D)
    cos_b = clamp(cos_b, -1.0, 1.0)
    beta = math.acos(cos_b)
    res['femur'] = (alpha - beta) * 180.0 / math.pi

    # Step 4: Tibia angle
    cos_g = (FEMUR_LEN**2 + TIBIA_LEN**2 - D**2) / (2.0 * FEMUR_LEN * TIBIA_LEN)
    cos_g = clamp(cos_g, -1.0, 1.0)
    res['tibia'] = math.acos(cos_g) * 180.0 / math.pi

    res['valid'] = True
    return res


# ─────────────────────────────────────────────
#  USER COMMAND FUNCTIONS (Call from REPL or CLI)
# ─────────────────────────────────────────────

def center(duration_ms=None):
    """Smoothly move all servos to 90° (L-shape neutral pose)."""
    print("Gliding to neutral (all servos 90°)...")
    move_smooth(90.0, 90.0, 90.0, duration_ms)


def neutral():
    center()


def angles(coxa, femur, tibia, duration_ms=None):
    """Smoothly move to raw servo angles in degrees."""
    print("Gliding to angles: coxa={:.1f}°, femur={:.1f}°, tibia={:.1f}°".format(coxa, femur, tibia))
    move_smooth(coxa, femur, tibia, duration_ms)


def foot(x, y, z, duration_ms=None):
    """Smoothly move foot to (x, y, z) mm in Cartesian space (straight foot path)."""
    print("Target foot: ({:.1f}, {:.1f}, {:.1f}) mm".format(x, y, z))
    ik = solve_ik(x, y, z)
    if ik['valid']:
        sc = ik_to_servo_coxa(ik['coxa'])
        sf = ik_to_servo_femur(ik['femur'])
        st = ik_to_servo_tibia(ik['tibia'])
        print("  IK geometric:  coxa={:.1f}°  femur={:.1f}°  tibia={:.1f}°".format(ik['coxa'], ik['femur'], ik['tibia']))
        print("  Servo outputs: coxa={:.1f}°  femur={:.1f}°  tibia={:.1f}°".format(sc, sf, st))
        apply_ik_result(ik, smooth_move=True, duration_ms=duration_ms)


def stand():
    """Smoothly move leg to default bent-knee standing pose."""
    print("Gliding to standing pose...")
    foot(STANDING_X, 0, STANDING_Z)


def lift():
    """Smoothly lift foot up from standing position."""
    print("Lifting foot...")
    foot(STANDING_X, 0, STANDING_Z + STEP_HEIGHT)


def sweep(joint=0):
    """Slowly sweep one joint (0=Coxa, 1=Femur, 2=Tibia). Press Ctrl+C to stop."""
    names = ["Coxa", "Femur", "Tibia"]
    joint = clamp(int(joint), 0, 2)
    print("Sweeping {} (joint {})... Press Ctrl+C to stop.".format(names[joint], joint))
    # First glide smoothly to 90° on the target joint
    targets = list(current_angles)
    targets[joint] = 90.0
    move_smooth(targets[0], targets[1], targets[2], 400)

    phase = 0.0
    dt = UPDATE_MS / 1000.0
    try:
        while True:
            phase += 2.0 * math.pi * (dt / 2.5)  # 2.5 seconds per full sine wave
            angle = 90.0 + 45.0 * math.sin(phase)
            write_joint_filtered(joint, angle)
            time.sleep_ms(UPDATE_MS)
    except KeyboardInterrupt:
        print("Sweep stopped.")


def sweep_full_range(joint, min_deg=10.0, max_deg=170.0, cycle_sec=3.0):
    """Continuously sweep one joint smoothly between min_deg and max_deg (10° to 170°)."""
    names = ["Coxa", "Femur", "Tibia"]
    joint = clamp(int(joint), 0, 2)
    mid = 0.5 * (min_deg + max_deg)
    amp = 0.5 * (max_deg - min_deg)

    print("Sweeping {} continuously from {:.0f}° to {:.0f}°... Press Ctrl+C to stop.".format(
        names[joint], min_deg, max_deg))

    # Smoothly glide that joint to center (90°) first so it doesn't snap
    targets = list(current_angles)
    targets[joint] = mid
    move_smooth(targets[0], targets[1], targets[2], 500)

    phase = 0.0
    dt = UPDATE_MS / 1000.0
    try:
        while True:
            phase += 2.0 * math.pi * (dt / cycle_sec)
            angle = mid + amp * math.sin(phase)
            write_joint_filtered(joint, angle)
            time.sleep_ms(UPDATE_MS)
    except KeyboardInterrupt:
        print("{} sweep stopped.".format(names[joint]))


def coxa():
    """Continuously sweep Coxa between 10° and 170°. Press Ctrl+C to stop."""
    sweep_full_range(0, 10.0, 170.0)


def femur():
    """Continuously sweep Femur between 10° and 170°. Press Ctrl+C to stop."""
    sweep_full_range(1, 10.0, 170.0)


def tibia():
    """Continuously sweep Tibia between 10° and 170°. Press Ctrl+C to stop."""
    sweep_full_range(2, 10.0, 170.0)


def walk():
    """Run ultra-smooth single-leg walking step cycle demo. Press Ctrl+C to stop."""
    print("Running smooth walk cycle demo... Press Ctrl+C to stop.")
    # Glide to the start of the swing phase first so there is no initial jerk!
    ik_start = solve_ik(STANDING_X - STRIDE_LENGTH, 0, STANDING_Z)
    if ik_start['valid']:
        apply_ik_result(ik_start, smooth_move=True, duration_ms=500)

    phase = 0.0
    dt = (UPDATE_MS / 1000.0) / WALK_CYCLE_SEC
    try:
        while True:
            phase = (phase + dt) % 1.0
            if phase < 0.5:
                # Swing phase (in air, moving forward with smooth cosine easing)
                t = phase / 0.5
                s = ease_in_out_cosine(t)  # Smooth horizontal acceleration/deceleration
                x = STANDING_X + STRIDE_LENGTH * (-1.0 + 2.0 * s)
                # Raised cosine bell curve for vertical lift (zero vertical velocity at liftoff & touchdown!)
                z = STANDING_Z + STEP_HEIGHT * 0.5 * (1.0 - math.cos(2.0 * math.pi * t))
            else:
                # Stance phase (on ground, moving backward)
                t = (phase - 0.5) / 0.5
                s = ease_in_out_cosine(t)
                x = STANDING_X + STRIDE_LENGTH * (1.0 - 2.0 * s)
                z = STANDING_Z

            ik = solve_ik(x, 0, z)
            if ik['valid']:
                apply_ik_result(ik, smooth_move=False)
            time.sleep_ms(UPDATE_MS)
    except KeyboardInterrupt:
        print("Walk demo stopped.")


def wave():
    """Wave the leg in a smooth circle. Press Ctrl+C to stop."""
    print("Running wave demo... Press Ctrl+C to stop.")
    # Glide to initial wave point first so it doesn't snap
    ik_start = solve_ik(STANDING_X, 0.0, STANDING_Z + STEP_HEIGHT + 25.0)
    if ik_start['valid']:
        apply_ik_result(ik_start, smooth_move=True, duration_ms=500)

    phase = 0.0
    dt = UPDATE_MS / 1000.0
    try:
        while True:
            phase += 2.0 * math.pi * (dt / 2.0)  # 2.0 seconds per circle
            x = STANDING_X
            y = 40.0 * math.sin(phase)
            z = STANDING_Z + STEP_HEIGHT + 25.0 * math.cos(phase)
            ik = solve_ik(x, y, z)
            if ik['valid']:
                apply_ik_result(ik, smooth_move=False)
            time.sleep_ms(UPDATE_MS)
    except KeyboardInterrupt:
        print("Wave demo stopped.")


def speed(cycle_sec=1.8, move_ms=600):
    """Adjust movement speed. Lower = faster, Higher = slower & gentler."""
    global WALK_CYCLE_SEC, MOVE_DURATION_MS
    WALK_CYCLE_SEC = max(0.4, float(cycle_sec))
    MOVE_DURATION_MS = max(100, int(move_ms))
    print("Updated speed: walk cycle = {:.2f}s, point-to-point glide = {}ms".format(
        WALK_CYCLE_SEC, MOVE_DURATION_MS))


def test_channel(ch):
    """Wiggle a raw PCA9685 channel (0-15) between 60° and 120° to identify which servo is connected."""
    ch = clamp(int(ch), 0, 15)
    print("Testing raw PCA9685 Channel {} (60° <-> 120°)... Press Ctrl+C to stop.".format(ch))
    phase = 0.0
    dt = UPDATE_MS / 1000.0
    try:
        while True:
            phase += 2.0 * math.pi * (dt / 1.5)
            angle = 90.0 + 30.0 * math.sin(phase)
            set_servo_pwm(ch, angle)
            time.sleep_ms(UPDATE_MS)
    except KeyboardInterrupt:
        set_servo_pwm(ch, 90.0)
        print("Channel {} returned to 90°.".format(ch))


def help():
    print("\n╔══════════════════════════════════════════════════════════╗")
    print("║     HexaPud Single Leg Test (Smooth Motion v2)           ║")
    print("╠══════════════════════════════════════════════════════════╣")
    print("║ REPL Functions / CLI Commands:                           ║")
    print("║   ch <0-15>        Wiggle raw PCA channel to ID servos   ║")
    print("║   coxa             Sweep Coxa  10° <-> 170° (Ctrl+C stop)║")
    print("║   femur            Sweep Femur 10° <-> 170° (Ctrl+C stop)║")
    print("║   tibia            Sweep Tibia 10° <-> 170° (Ctrl+C stop)║")
    print("║   center           Smooth glide to 90° (L-shape neutral) ║")
    print("║   stand            Smooth glide to standing pose         ║")
    print("║   lift             Smoothly lift foot up from stand      ║")
    print("║   angles c f t     Smooth glide to servo angles (°)      ║")
    print("║   foot x y z       Smooth glide to foot pos via IK (mm)  ║")
    print("║   sweep <0|1|2>    Sweep joint 45° <-> 135°              ║")
    print("║   walk             Smooth step cycle (Ctrl+C to stop)    ║")
    print("║   wave             Smooth wave loop (Ctrl+C to stop)     ║")
    print("║   speed <sec> <ms> Set walk cycle sec & glide time (ms)  ║")
    print("╚══════════════════════════════════════════════════════════╝\n")


def cli():
    """Interactive text command loop over Serial."""
    help()
    while True:
        try:
            cmd = input("HexaPud> ").strip()
        except (EOFError, KeyboardInterrupt):
            print("\nExiting CLI. You can call functions directly from REPL.")
            break

        if not cmd:
            continue
        parts = cmd.split()
        action = parts[0].lower()

        try:
            if action in ("help", "?"):
                help()
            elif action in ("center", "neutral"):
                center()
            elif action == "ch" and len(parts) > 1:
                test_channel(int(parts[1]))
            elif action == "coxa":
                coxa()
            elif action == "femur":
                femur()
            elif action == "tibia":
                tibia()
            elif action == "stand":
                stand()
            elif action == "lift":
                lift()
            elif action == "walk":
                walk()
            elif action == "wave":
                wave()
            elif action == "sweep":
                j = int(parts[1]) if len(parts) > 1 else 0
                sweep(j)
            elif action == "angles" and len(parts) >= 4:
                dur = int(parts[4]) if len(parts) >= 5 else None
                angles(float(parts[1]), float(parts[2]), float(parts[3]), dur)
            elif action == "foot" and len(parts) >= 4:
                dur = int(parts[4]) if len(parts) >= 5 else None
                foot(float(parts[1]), float(parts[2]), float(parts[3]), dur)
            elif action == "speed":
                sec = float(parts[1]) if len(parts) > 1 else 1.8
                ms = int(parts[2]) if len(parts) > 2 else 600
                speed(sec, ms)
            elif action in ("exit", "quit"):
                break
            else:
                print("Unknown command or wrong arguments. Type 'help'.")
        except Exception as e:
            print("Error:", e)


# ─────────────────────────────────────────────
#  BOOT SELF-TEST
# ─────────────────────────────────────────────

print("\n══════════════════════════════════════════════════")
print("  HexaPud — Single Leg Test (Smooth Motion v2)")
print("══════════════════════════════════════════════════")
print("Verifying IK at neutral pose (131, 0, -128):")
_test = solve_ik(COXA_LEN + FEMUR_LEN, 0, -TIBIA_LEN)
if _test['valid']:
    print("  IK geometric:  coxa={:.1f}°  femur={:.1f}°  tibia={:.1f}°".format(
        _test['coxa'], _test['femur'], _test['tibia']))
    print("  Servo outputs: coxa={:.1f}°  femur={:.1f}°  tibia={:.1f}°".format(
        ik_to_servo_coxa(_test['coxa']), ik_to_servo_femur(_test['femur']), ik_to_servo_tibia(_test['tibia'])))

# On initial boot, set servos to 90° with 100ms stagger so they don't jolt
write_joint_direct(0, 90.0)
time.sleep_ms(100)
write_joint_direct(1, 90.0)
time.sleep_ms(100)
write_joint_direct(2, 90.0)

print("\nReady! Starting interactive CLI (press Ctrl+C to drop to standard Python REPL)...")
if __name__ == "__main__":
    cli()
