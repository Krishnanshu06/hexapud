"""
HexaPud - 6-Leg Test and Calibration Controller (MicroPython)
Interactive REPL utility for direct joint nudging, pose keyframing, and tripod walk tests.
ESP32 + Dual PCA9685 (0x40 & 0x41) + 18 MG996R Servos
"""

from machine import Pin, I2C
import math
import time
from pca9685 import PCA9685

# --- Hardware & I2C Configuration ---
I2C_SDA = 21
I2C_SCL = 23
PCA1_ADDR = 0x40   # Board 1: Left legs (0, 1, 2)
PCA2_ADDR = 0x41   # Board 2: Right legs (3, 4, 5)

SERVO_MIN_US = 500
SERVO_MAX_US = 2500

UPDATE_MS = 20         # 50Hz update rate
DEFAULT_MOVE_MS = 350
SPEED_SCALE = 1.0

# --- Leg Configurations & Offsets ---
# Each leg has:
#   name: human readable string
#   board: 1 (0x40) or 2 (0x41)
#   ch_coxa, ch_femur, ch_tibia: PCA channel on that board
#   mount_angle: base hexagon angle from front (0°)
#   coxa_dir, femur_dir, tibia_dir: 1 or -1
#   coxa_off, femur_off, tibia_off: calibration angle offsets in degrees

# Note on Tibia offsets:
#   Leg 0 (White PLA) mounted at 90° -> offset = 0.0
#   Legs 1..5 (Black PLA) mounted at ~75° -> offset = +15.0° (brings logical 90 to physical 90)
LEG_CONFIGS = [
    {
        "name": "Front-Left (White PLA)",
        "board": 1, "ch_coxa": 0, "ch_femur": 1, "ch_tibia": 2,
        "mount_angle": 30.0,
        "dir": [1, 1, 1],
        "offset": [5.0, 10.0, -3.0]        # White leg is calibrated at 90°
    },
    {
        "name": "Middle-Left (Black PLA)",
        "board": 1, "ch_coxa": 4, "ch_femur": 5, "ch_tibia": 6,
        "mount_angle": 90.0,
        "dir": [1, 1, 1],
        "offset": [0, -2.0, -21.0]       # 75° physical + 15° = 90° neutral
    },
    {
        "name": "Back-Left (Black PLA)",
        "board": 1, "ch_coxa": 8, "ch_femur": 9, "ch_tibia": 10,
        "mount_angle": 150.0,
        "dir": [1, 1, 1],
        "offset": [0.0, 5.0, -15.0]
    },
    {
        "name": "Back-Right (Black PLA)",
        "board": 2, "ch_coxa": 0, "ch_femur": 1, "ch_tibia": 2,
        "mount_angle": -150.0,
        "dir": [-1, -1, -1],               # Mirrored side: coxa direction inverted
        "offset": [0.0, -14.0, 3.0]
    },
    {
        "name": "Middle-Right (Black PLA)",
        "board": 2, "ch_coxa": 4, "ch_femur": 5, "ch_tibia": 6,
        "mount_angle": -90.0,
        "dir": [-1, -1, -1],
        "offset": [-8.0, -5.0, 13.0]
    },
    {
        "name": "Front-Right (Black PLA)",
        "board": 2, "ch_coxa": 8, "ch_femur": 9, "ch_tibia": 10,
        "mount_angle": -30.0,
        "dir": [-1, -1, -1],
        "offset": [-7.0, -7.0, 4.0]
    }
]

# Leg Dimensions (mm)
COXA_LEN  = 51.0
FEMUR_LEN = 80.0
TIBIA_LEN = 128.0

STANDING_X = 130.0   # mm outward from coxa
STANDING_Z = -140.0  # mm below coxa

# Current logical angles for all 6 legs: 6 legs x 3 joints
current_angles = [[90.0, 90.0, 90.0] for _ in range(6)]

# ─────────────────────────────────────────────
#  INITIALIZATION & I2C SCAN
# ─────────────────────────────────────────────

time.sleep_ms(100)
# Use 400kHz Fast-mode I2C (4x faster bus speed so 18 servos update in ~2ms instead of 10ms)
i2c = I2C(0, scl=Pin(I2C_SCL), sda=Pin(I2C_SDA), freq=400000)
detected = i2c.scan()
print("I2C Bus Scan found:", [hex(d) for d in detected])

pwm1 = None
pwm2 = None

if PCA1_ADDR in detected:
    pwm1 = PCA9685(i2c, address=PCA1_ADDR)
    pwm1.set_pwm_freq(50)
    print("  [OK] Board 1 (Left Legs 0, 1, 2) initialized at 0x{:02X}".format(PCA1_ADDR))
else:
    print("  [WARNING] Board 1 (0x{:02X}) NOT found!".format(PCA1_ADDR))

if PCA2_ADDR in detected:
    pwm2 = PCA9685(i2c, address=PCA2_ADDR)
    pwm2.set_pwm_freq(50)
    print("  [OK] Board 2 (Right Legs 3, 4, 5) initialized at 0x{:02X}".format(PCA2_ADDR))
else:
    print("  [WARNING] Board 2 (0x{:02X}) NOT found (check solder bridge on A0 / power)".format(PCA2_ADDR))


def clamp(val, min_val, max_val):
    return max(min_val, min(max_val, val))


def ease_in_out_cosine(t):
    t = clamp(t, 0.0, 1.0)
    return 0.5 * (1.0 - math.cos(math.pi * t))


# ─────────────────────────────────────────────
#  LOW LEVEL SERVO WRITING
# ─────────────────────────────────────────────

def _write_raw_hw(board_id, channel, angle_deg):
    angle = clamp(angle_deg, 10.0, 170.0)
    pulse_us = SERVO_MIN_US + (angle / 180.0) * (SERVO_MAX_US - SERVO_MIN_US)
    if board_id == 1 and pwm1:
        pwm1.set_servo_pulse(channel, pulse_us)
    elif board_id == 2 and pwm2:
        pwm2.set_servo_pulse(channel, pulse_us)


def write_leg_joint(leg_idx, joint_idx, logical_angle):
    """
    Applies direction inversion, calibration offset, clamps safety limits,
    and sends PWM to the proper PCA board and channel.
    """
    cfg = LEG_CONFIGS[leg_idx]
    d = cfg["dir"][joint_idx]
    off = cfg["offset"][joint_idx]
    ch = [cfg["ch_coxa"], cfg["ch_femur"], cfg["ch_tibia"]][joint_idx]

    hw_angle = (180.0 - logical_angle) if d == -1 else logical_angle
    hw_angle += off
    hw_angle = clamp(hw_angle, 10.0, 170.0)

    current_angles[leg_idx][joint_idx] = logical_angle
    _write_raw_hw(cfg["board"], ch, hw_angle)


# ─────────────────────────────────────────────
#  INVERSE KINEMATICS SOLVER
# ─────────────────────────────────────────────

def solve_ik(x, y, z):
    """Solve 3-DOF geometric IK for single leg coordinates (mm)."""
    res = {'coxa': 0.0, 'femur': 0.0, 'tibia': 0.0, 'valid': False}
    res['coxa'] = math.atan2(y, x) * 180.0 / math.pi

    horiz_dist = math.sqrt(x * x + y * y)
    d_horiz = horiz_dist - COXA_LEN
    if d_horiz < 0:
        return res

    D = math.sqrt(d_horiz * d_horiz + z * z)
    max_reach = FEMUR_LEN + TIBIA_LEN
    min_reach = abs(FEMUR_LEN - TIBIA_LEN)
    if D > max_reach or D < min_reach:
        return res

    alpha = math.atan2(-z, d_horiz)
    cos_b = (FEMUR_LEN**2 + D**2 - TIBIA_LEN**2) / (2.0 * FEMUR_LEN * D)
    cos_b = clamp(cos_b, -1.0, 1.0)
    beta = math.acos(cos_b)
    res['femur'] = (alpha - beta) * 180.0 / math.pi

    cos_g = (FEMUR_LEN**2 + TIBIA_LEN**2 - D**2) / (2.0 * FEMUR_LEN * TIBIA_LEN)
    cos_g = clamp(cos_g, -1.0, 1.0)
    res['tibia'] = math.acos(cos_g) * 180.0 / math.pi

    res['valid'] = True
    return res


def ik_to_servo(ik_res):
    """Convert geometric angles to logical servo angles (90 neutral)."""
    return [
        90.0 + ik_res['coxa'],
        90.0 + ik_res['femur'],
        ik_res['tibia']
    ]


# ─────────────────────────────────────────────
#  SMOOTH MOVEMENT ENGINE
# ─────────────────────────────────────────────

def move_all_smooth(target_all_angles, duration_ms=DEFAULT_MOVE_MS):
    """
    Smoothly glide all 18 servos together using S-curve interpolation.
    target_all_angles: list of 6 legs, each [coxa, femur, tibia]
    """
    duration_ms = max(50, int(duration_ms / SPEED_SCALE))
    start_angles = [[current_angles[l][j] for j in range(3)] for l in range(6)]
    steps = max(1, int(duration_ms // UPDATE_MS))

    for s_idx in range(1, steps + 1):
        s = ease_in_out_cosine(s_idx / steps)
        for l in range(6):
            for j in range(3):
                interp = start_angles[l][j] + (target_all_angles[l][j] - start_angles[l][j]) * s
                write_leg_joint(l, j, interp)
        time.sleep_ms(UPDATE_MS)


def move_leg_smooth(leg_idx, target_joint_angles, duration_ms=DEFAULT_MOVE_MS):
    """Smoothly glide only 1 specific leg (3 joints)."""
    duration_ms = max(50, int(duration_ms / SPEED_SCALE))
    start_angles = [current_angles[leg_idx][j] for j in range(3)]
    steps = max(1, int(duration_ms // UPDATE_MS))

    for s_idx in range(1, steps + 1):
        s = ease_in_out_cosine(s_idx / steps)
        for j in range(3):
            interp = start_angles[j] + (target_joint_angles[j] - start_angles[j]) * s
            write_leg_joint(leg_idx, j, interp)
        time.sleep_ms(UPDATE_MS)


# ─────────────────────────────────────────────
#  HIGH LEVEL POSES & CONTROL COMMANDS
# ─────────────────────────────────────────────

def center(duration_ms=DEFAULT_MOVE_MS):
    """Move all 6 legs (18 servos) to neutral 90°."""
    print("Centering all 6 legs to 90°...")
    targets = [[90.0, 90.0, 90.0] for _ in range(6)]
    move_all_smooth(targets, duration_ms)


def stand(duration_ms=DEFAULT_MOVE_MS):
    """Move all 6 legs into stable bent-knee standing pose."""
    print("Moving all 6 legs to standing pose...")
    ik = solve_ik(STANDING_X, 0, STANDING_Z)
    if not ik['valid']:
        print("IK error on standing pose")
        return
    angles = ik_to_servo(ik)
    targets = [list(angles) for _ in range(6)]
    move_all_smooth(targets, duration_ms)


def leg_angles(leg_idx, coxa, femur, tibia, duration_ms=DEFAULT_MOVE_MS):
    """Set raw logical angles for one leg."""
    if 0 <= leg_idx <= 5:
        print("Setting Leg {} ({}) to ({:.1f}, {:.1f}, {:.1f})".format(
            leg_idx, LEG_CONFIGS[leg_idx]["name"], coxa, femur, tibia))
        move_leg_smooth(leg_idx, [coxa, femur, tibia], duration_ms)


def leg_foot(leg_idx, x, y, z, duration_ms=DEFAULT_MOVE_MS):
    """Set Cartesian foot position for one leg via IK."""
    if 0 <= leg_idx <= 5:
        ik = solve_ik(x, y, z)
        if ik['valid']:
            angles = ik_to_servo(ik)
            print("Leg {} IK target: coxa={:.1f}° femur={:.1f}° tibia={:.1f}°".format(
                leg_idx, angles[0], angles[1], angles[2]))
            move_leg_smooth(leg_idx, angles, duration_ms)
        else:
            print("Unreachable foot position for leg {}".format(leg_idx))


def test_channel(board_id, ch):
    """Wiggle raw channel on Board 1 or 2 to trace wiring."""
    print("Testing Board {} Channel {} (60° <-> 120°)... Press Ctrl+C to stop.".format(board_id, ch))
    phase = 0.0
    dt = UPDATE_MS / 1000.0
    try:
        while True:
            phase += 2.0 * math.pi * (dt / 1.5)
            angle = 90.0 + 30.0 * math.sin(phase)
            _write_raw_hw(board_id, ch, angle)
            time.sleep_ms(UPDATE_MS)
    except KeyboardInterrupt:
        _write_raw_hw(board_id, ch, 90.0)
        print("Test stopped.")


def test_leg(leg_idx):
    """Gently wiggles Coxa, Femur, and Tibia of one leg in sequence to confirm joint order."""
    if not (0 <= leg_idx <= 5):
        return
    print("\nTesting Leg {} ({}) joints one by one:".format(leg_idx, LEG_CONFIGS[leg_idx]["name"]))

    names = ["Coxa (Hip)", "Femur (Thigh)", "Tibia (Shin)"]
    for j in range(3):
        print("  -> Wiggling {}".format(names[j]))
        curr = list(current_angles[leg_idx])
        curr[j] = 115.0
        move_leg_smooth(leg_idx, curr, 300)
        curr[j] = 65.0
        move_leg_smooth(leg_idx, curr, 400)
        curr[j] = 90.0
        move_leg_smooth(leg_idx, curr, 300)
        time.sleep_ms(150)
    print("Done testing Leg {}.\n".format(leg_idx))


def test_all_sequential():
    """Tests all 6 legs one after another to verify wiring and current draw."""
    print("Testing all 6 legs in sequence (0 through 5)...")
    for l in range(6):
        test_leg(l)


def set_offset(leg_idx, joint_idx, offset_deg):
    """Adjust calibration offset live (0=coxa, 1=femur, 2=tibia)."""
    if 0 <= leg_idx <= 5 and 0 <= joint_idx <= 2:
        LEG_CONFIGS[leg_idx]["offset"][joint_idx] = float(offset_deg)
        print("Leg {} joint {} offset updated to {:.1f}°".format(leg_idx, joint_idx, offset_deg))
        # Refresh current pose with new offset
        write_leg_joint(leg_idx, joint_idx, current_angles[leg_idx][joint_idx])


def wave_all():
    """Gently ripple / wave all 6 legs in a circle (demonstrates power delivery)."""
    print("Waving all 6 legs... Press Ctrl+C to stop.")
    phase = 0.0
    dt = UPDATE_MS / 1000.0
    try:
        while True:
            phase += 2.0 * math.pi * (dt / 2.0)
            for l in range(6):
                leg_phase = phase + (l * math.pi / 3.0)  # 60° phase offset per leg
                x = STANDING_X
                y = 20.0 * math.sin(leg_phase)
                z = STANDING_Z + 30.0 + 20.0 * math.cos(leg_phase)
                ik = solve_ik(x, y, z)
                if ik['valid']:
                    s = ik_to_servo(ik)
                    for j in range(3):
                        write_leg_joint(l, j, s[j])
            time.sleep_ms(UPDATE_MS)
    except KeyboardInterrupt:
        print("Wave stopped.")
        stand(500)


# ─────────────────────────────────────────────
#  KEYFRAME ANIMATION ENGINE
# ─────────────────────────────────────────────

def play_keyframes(keyframe_list):
    """
    Plays a list of keyframe poses sequentially with smooth S-curve interpolation.
    Each keyframe is: (pose_name, target_angles_6x3, duration_ms)
      - target_angles_6x3: either a 6x3 list of angles, OR a 3-element list [c, f, t]
        applied to all 6 legs simultaneously.
    """
    for name, pose, dur in keyframe_list:
        print("  -> Phase: {} ({}ms)...".format(name, dur))
        if len(pose) == 3 and not isinstance(pose[0], (list, tuple)):
            # Broadcast [c, f, t] to all 6 legs
            target_6x3 = [list(pose) for _ in range(6)]
        else:
            target_6x3 = pose
        move_all_smooth(target_6x3, dur)
        time.sleep_ms(20)


# ─────────────────────────────────────────────
#  CHOREOGRAPHED LIFELIKE MOVEMENTS
# ─────────────────────────────────────────────

def flat_pose(duration_ms=DEFAULT_MOVE_MS):
    """Legs laid completely straight out / flat on the table."""
    print("Moving to flat pose (legs laid straight out)...")
    # Coxa straight (90°), Femur horizontal (90°), Tibia unfolded straight (~20°)
    move_all_smooth([[90.0, 90.0, 20.0] for _ in range(6)], duration_ms)


def wakeup():
    """
    Wakes up quickly and snappily from flat lay-down pose:
      1. Starts flat on table (belly on floor, legs out).
      2. Tilts femurs UP into the air (knees lift).
      3. Drops tibias straight DOWN towards the table.
      4. Lowers femurs down to push body up into full standing stance!
    """
    print("\n[Animation] Waking up from flat pose (Fast & Snappy)...")
    ik_stand = solve_ik(STANDING_X, 0, STANDING_Z)
    stand_angles = ik_to_servo(ik_stand) if ik_stand['valid'] else [90.0, 99.0, 99.0]

    sequence = [
        ("1. Flat on floor",      [90.0,  90.0,  20.0],  300),
        ("2. Knees lift up",      [90.0, 135.0,  25.0],  350),  # Quick lift
        ("3. Tibias deploy down", [90.0, 135.0, 115.0],  350),  # Quick snap down
        ("4. Push body up",       stand_angles,          450)   # Quick stand
    ]
    play_keyframes(sequence)
    print("[Animation] Wake-up complete! Standing tall.\n")


def sleep():
    """
    Fast, graceful sleep onto chassis.
    """
    print("\n[Animation] Going to sleep...")
    sequence = [
        ("1. Unload legs",        [90.0, 120.0, 100.0],  300),
        ("2. Extend tibias out",  [90.0, 115.0,  35.0],  350),
        ("3. Lower belly down",   [90.0,  95.0,  20.0],  350),
        ("4. Relax flat",         [90.0,  90.0,  20.0],  250)
    ]
    play_keyframes(sequence)
    print("[Animation] Robot is asleep / resting on chassis.\n")


def pushups(count=3):
    """
    Fast, energetic push-ups.
    """
    print("\n[Animation] Starting {} push-ups...".format(count))
    z_low = STANDING_Z + 45.0
    z_high = STANDING_Z - 20.0

    ik_low = solve_ik(STANDING_X + 15.0, 0, z_low)
    ik_high = solve_ik(STANDING_X, 0, z_high)
    if not (ik_low['valid'] and ik_high['valid']):
        print("IK error on pushups height")
        return

    low_angles = ik_to_servo(ik_low)
    high_angles = ik_to_servo(ik_high)

    move_all_smooth([high_angles for _ in range(6)], 250)
    time.sleep_ms(80)

    for i in range(count):
        print("  Push-up {}/{}...".format(i + 1, count))
        move_all_smooth([low_angles for _ in range(6)], 250)   # Fast drop
        time.sleep_ms(50)
        move_all_smooth([high_angles for _ in range(6)], 220)  # Explosive press up!
        time.sleep_ms(50)

    stand(300)
    print("[Animation] Push-ups complete!\n")


def body_twist():
    """
    Keeps all 6 feet glued to their positions on the floor while rotating
    and pitching the body in 3D (shows off multi-axis IK body dynamics!).
    """
    print("\n[Animation] Body pitch, roll & yaw twist demo... Press Ctrl+C to stop.")
    phase = 0.0
    dt = UPDATE_MS / 1000.0
    cycle_time = 2.5  # seconds per full swirl

    try:
        while True:
            phase += 2.0 * math.pi * (dt / cycle_time)
            # Circular tilt/yaw offsets
            shift_x = 18.0 * math.cos(phase)
            shift_y = 18.0 * math.sin(phase)
            shift_z = 10.0 * math.sin(phase * 2.0)

            all_targets = []
            for l in range(6):
                mount_rad = math.radians(LEG_CONFIGS[l]["mount_angle"])
                # Project body shift into local leg coordinate system
                leg_local_x = STANDING_X - (shift_x * math.cos(mount_rad) + shift_y * math.sin(mount_rad))
                leg_local_y = -(shift_y * math.cos(mount_rad) - shift_x * math.sin(mount_rad))
                leg_local_z = STANDING_Z + shift_z

                ik = solve_ik(leg_local_x, leg_local_y, leg_local_z)
                if ik['valid']:
                    all_targets.append(ik_to_servo(ik))
                else:
                    all_targets.append(current_angles[l])

            for l in range(6):
                for j in range(3):
                    write_leg_joint(l, j, all_targets[l][j])
            time.sleep_ms(UPDATE_MS)
    except KeyboardInterrupt:
        print("Body twist stopped.")
        stand(600)


def servo(leg_idx, joint_name_or_idx, target_angle, dur_ms=250):
    """
    Set a single servo on a specific leg to an exact custom angle.
      leg_idx: 0 to 5
      joint_name_or_idx: 0/'c'/'coxa', 1/'f'/'femur', 2/'t'/'tibia'
      target_angle: angle in degrees (10 to 170)
    """
    if isinstance(joint_name_or_idx, str):
        j_str = joint_name_or_idx.lower()
        if j_str in ('c', 'coxa', 'hip'):
            j_idx = 0
        elif j_str in ('f', 'femur', 'thigh'):
            j_idx = 1
        elif j_str in ('t', 'tibia', 'shin', 'knee'):
            j_idx = 2
        else:
            j_idx = int(joint_name_or_idx)
    else:
        j_idx = int(joint_name_or_idx)

    if not (0 <= leg_idx <= 5 and 0 <= j_idx <= 2):
        print("Invalid leg or joint index!")
        return

    names = ["Coxa", "Femur", "Tibia"]
    target_angle = clamp(float(target_angle), 10.0, 170.0)
    print("Moving Leg {} ({}) {} to {:.1f}°...".format(
        leg_idx, LEG_CONFIGS[leg_idx]["name"], names[j_idx], target_angle))

    curr_target = list(current_angles[leg_idx])
    curr_target[j_idx] = target_angle
    move_leg_smooth(leg_idx, curr_target, dur_ms)


def raw_all(coxa_angle, femur_angle, tibia_angle, dur_ms=350):
    """
    Send all 6 legs to the same [coxa, femur, tibia] angles simultaneously.
    e.g. raw_all 90 90 20 (useful for testing flat/sleep pose across all legs).
    """
    c = clamp(float(coxa_angle), 10.0, 170.0)
    f = clamp(float(femur_angle), 10.0, 170.0)
    t = clamp(float(tibia_angle), 10.0, 170.0)
    print("Moving ALL 6 legs to coxa={:.1f}°, femur={:.1f}°, tibia={:.1f}°...".format(c, f, t))
    move_all_smooth([[c, f, t] for _ in range(6)], dur_ms)


def print_offsets():
    """Print the full calibration table of all 6 legs with current offsets."""
    print("\n╔════════════════════════════════════════════════════════════════╗")
    print("║            HexaPud Leg Offsets & Current Angles Table          ║")
    print("╠══════╦══════════════════════╦═════════════════╦════════════════╣")
    print("║ Leg  ║ Name                 ║ Offsets (C,F,T) ║ Angles (C,F,T) ║")
    print("╠══════╬══════════════════════╬═════════════════╬════════════════╣")
    for l in range(6):
        cfg = LEG_CONFIGS[l]
        o = cfg["offset"]
        a = current_angles[l]
        print("║  {:1d}   ║ {:20s} ║ [{:4.1f},{:4.1f},{:4.1f}] ║ [{:3.0f},{:3.0f},{:3.0f}]   ║".format(
            l, cfg["name"][:20], o[0], o[1], o[2], a[0], a[1], a[2]))
    print("╚══════╩══════════════════════╩═════════════════╩════════════════╝\n")


def tripod(group='a', duration_ms=400):
    """
    Stand stably on 3 legs while lifting the other 3 into the air!
      group='a' -> Tripod A stands (Legs 0, 2, 4), Tripod B lifts (Legs 1, 3, 5)
      group='b' -> Tripod B stands (Legs 1, 3, 5), Tripod A lifts (Legs 0, 2, 4)
    """
    grp = str(group).lower().strip()
    if grp in ('a', '0'):
        standing_legs = [0, 2, 4]  # Front-Left, Back-Left, Middle-Right
        lifted_legs   = [1, 3, 5]  # Middle-Left, Back-Right, Front-Right
        name = "Tripod A"
    elif grp in ('b', '1'):
        standing_legs = [1, 3, 5]  # Middle-Left, Back-Right, Front-Right
        standing_legs = [1, 3, 5]
        lifted_legs   = [0, 2, 4]  # Front-Left, Back-Left, Middle-Right
        name = "Tripod B"
    else:
        print("Invalid tripod group! Use 'tripod a' or 'tripod b'.")
        return

    print("\n[Pose] Standing on {} (Legs {}) while lifting Legs {}...".format(
        name, standing_legs, lifted_legs))

    # Calculate standing IK
    ik_stand = solve_ik(STANDING_X, 0, STANDING_Z)
    stand_ang = ik_to_servo(ik_stand) if ik_stand['valid'] else [90.0, 99.0, 99.0]

    # Calculate lifted foot (lift foot 45mm higher into air)
    ik_lift = solve_ik(STANDING_X - 15.0, 0, STANDING_Z + 45.0)
    lift_ang = ik_to_servo(ik_lift) if ik_lift['valid'] else [90.0, 125.0, 70.0]

    target_6x3 = []
    for l in range(6):
        if l in standing_legs:
            target_6x3.append(list(stand_ang))
        else:
            target_6x3.append(list(lift_ang))

    move_all_smooth(target_6x3, duration_ms)
    print("[Pose] Tripod stance complete!\n")


def toggle_tripod(count=4, interval_ms=600):
    """
    Rhythmically alternate standing between Tripod A and Tripod B.
    This is the core foundation of walking!
    """
    print("\n[Demo] Alternating Tripod A <-> Tripod B {} times...".format(count))
    stand(350)
    time.sleep_ms(200)

    for i in range(count):
        print("  -> Step {}: Tripod A standing".format(i * 2 + 1))
        tripod('a', interval_ms)
        time.sleep_ms(100)
        print("  -> Step {}: Tripod B standing".format(i * 2 + 2))
        tripod('b', interval_ms)
        time.sleep_ms(100)

    stand(350)
    print("[Demo] Tripod alternating complete!\n")


def speed(multiplier=1.0):
    """
    Set overall animation speed multiplier live from CLI or REPL.
    e.g. speed 1.5 (50% faster), speed 2.0 (2x fast/snappy), speed 0.5 (slower)
    """
    global SPEED_SCALE
    SPEED_SCALE = max(0.2, min(5.0, float(multiplier)))
    print("Animation speed multiplier set to {:.2f}x".format(SPEED_SCALE))


# ─────────────────────────────────────────────
#  CONTROLLED 5-SECOND WALKING TEST
# ─────────────────────────────────────────────

def walk(seconds=5.0, direction="forward", step_len=40.0, step_lift=34.0):
    """
    Executes a clean, smooth tripod gait for exactly N seconds.
      seconds: duration of walk in seconds (default 5.0)
      direction: 'forward', 'backward', 'left', 'right'
    """
    print("\n" + "=" * 55)
    print("  ★ STARTING {:.1f}-SECOND TRIPOD WALK ({})".format(seconds, direction.upper()))
    print("=" * 55)

    # Move to stand first to establish baseline
    stand(350)
    time.sleep_ms(200)

    # Direction vectors for all 8 cardinal & diagonal headings
    diag = step_len * 0.7071  # cos(45°) = 0.7071
    dirs = {
        "forward":    ( step_len,  0.0),
        "f":          ( step_len,  0.0),
        "backward":   (-step_len,  0.0),
        "b":          (-step_len,  0.0),
        "left":       ( 0.0,       step_len),
        "l":          ( 0.0,       step_len),
        "right":      ( 0.0,      -step_len),
        "r":          ( 0.0,      -step_len),
        "fl":         ( diag,      diag),       # Forward-Left
        "forward_left": ( diag,    diag),
        "fr":         ( diag,     -diag),       # Forward-Right
        "forward_right":( diag,   -diag),
        "bl":         (-diag,      diag),       # Backward-Left
        "backward_left":(-diag,    diag),
        "br":         (-diag,     -diag),       # Backward-Right
        "backward_right":(-diag,  -diag)
    }

    dir_key = direction.lower().strip()
    if dir_key in dirs:
        world_dx, world_dy = dirs[dir_key]
    else:
        print("Unknown direction '{}', defaulting to forward. Options: f, b, l, r, fl, fr, bl, br".format(direction))
        world_dx, world_dy = step_len, 0.0

    cycle_time_sec = 0.9   # 0.9s per full tripod stride
    gait_speed = (UPDATE_MS / 1000.0) / cycle_time_sec
    gait_phase = 0.0

    total_ticks = int((seconds * 1000.0) / UPDATE_MS)
    print("Executing {} ticks (~{:.1f} strides)...".format(total_ticks, seconds / cycle_time_sec))

    try:
        for t_idx in range(total_ticks):
            gait_phase = (gait_phase + gait_speed) % 1.0

            for l in range(6):
                is_tripod_a = (l in (0, 2, 4))
                leg_p = gait_phase if is_tripod_a else ((gait_phase + 0.5) % 1.0)
                m_rad = math.radians(LEG_CONFIGS[l]["mount_angle"])
                is_right_side = (l in (3, 4, 5))

                # Project world vector into leg frame with correct right-side hardware mirroring
                raw_lx = world_dx * math.cos(m_rad) + world_dy * math.sin(m_rad)
                raw_ly = -world_dx * math.sin(m_rad) + world_dy * math.cos(m_rad)

                local_dir_x = raw_lx * 0.5
                local_dir_y = (-raw_ly if is_right_side else raw_ly) * 0.5

                if leg_p < 0.5:
                    # ── SWING (In air lifting and stepping forward) ──
                    t = leg_p / 0.5
                    s = ease_in_out_cosine(t)
                    lz = STANDING_Z + step_lift * math.sin(math.pi * t)
                    p = -1.0 + 2.0 * s
                else:
                    # ── STANCE (On ground pushing chassis forward) ──
                    t = (leg_p - 0.5) / 0.5
                    s = ease_in_out_cosine(t)
                    lz = STANDING_Z
                    p = 1.0 - 2.0 * s

                lx = STANDING_X + p * local_dir_x
                ly = p * local_dir_y

                ik = solve_ik(lx, ly, lz)
                if ik['valid']:
                    s_angles = ik_to_servo(ik)
                    for j in range(3):
                        write_leg_joint(l, j, s_angles[j])

            time.sleep_ms(UPDATE_MS)

    except KeyboardInterrupt:
        print("\nWalk interrupted by user!")

    # Return smoothly to stable stand
    stand(300)
    print("Walk finished. Returned to stand.\n")


def help():
    print("\n╔════════════════════════════════════════════════════════════════╗")
    print("║          HexaPud 6-Leg Test & Calibration System               ║")
    print("╠════════════════════════════════════════════════════════════════╣")
    print("║ Animations & Expressions (Fast & Snappy):                      ║")
    print("║   walk [sec] [dir]     8-Way Walk (f, b, l, r, fl, fr, bl, br) ║")
    print("║   tripod <a|b>         Stand on 3 legs & lift other 3 in air   ║")
    print("║   alttripod [N]        Alternate Tripod A <-> B (walk prep)    ║")
    print("║   wakeup               Life-like wake up from flat-lay pose    ║")
    print("║   sleep                Gently lay down flat onto chassis       ║")
    print("║   flat                 Lay all legs straight out flat          ║")
    print("║   pushups [N]          Do N body push-ups (e.g. pushups 3)     ║")
    print("║   twist                3D body roll/pitch/yaw swirl demo       ║")
    print("║   wave                 Continuous ripple across all legs       ║")
    print("║   speed <mult>         Change speed live (e.g. speed 2.0)      ║")
    print("║                                                                ║")
    print("║ Calibration & Positioning:                                     ║")
    print("║   servo <leg> <j> <deg> Set 1 servo angle (j: 0/c, 1/f, 2/t)    ║")
    print("║   rawall <c> <f> <t>   Set all 6 legs to custom raw angles     ║")
    print("║   offsets              Display table of all leg offsets        ║")
    print("║   offset <leg> <j> <deg> Set offset (j: 0=coxa, 1=fem, 2=tib)  ║")
    print("║   center               Smoothly glide all 18 servos to 90°     ║")
    print("║   stand                All 6 legs stand in stable pose         ║")
    print("║   testleg <0-5>        Wiggle joints of one leg (Coxa/Fem/Tib) ║")
    print("║   testall              Wiggle all 6 legs one by one            ║")
    print("║   ch <board 1|2> <ch>  Wiggle single raw PCA9685 pin (0-15)    ║")
    print("║   leg <0-5> c f t      Set angles for one leg (e.g. leg 0 90 90)║")
    print("║   foot <0-5> x y z     Set foot pos via IK (e.g. foot 0 130 0) ║")
    print("╚════════════════════════════════════════════════════════════════╝\n")


def cli():
    help()
    while True:
        try:
            cmd = input("HexaPud-6L> ").strip()
        except (EOFError, KeyboardInterrupt):
            print("\nExiting CLI.")
            break

        if not cmd:
            continue
        parts = cmd.split()
        act = parts[0].lower()

        try:
            if act in ("help", "?"):
                help()
            elif act in ("center", "neutral"):
                center()
            elif act == "stand":
                stand()
            elif act == "walk":
                sec = float(parts[1]) if len(parts) > 1 else 5.0
                direction = parts[2].lower() if len(parts) > 2 else "forward"
                walk(sec, direction)
            elif act == "tripod":
                grp = parts[1] if len(parts) > 1 else 'a'
                dur = int(parts[2]) if len(parts) > 2 else 400
                tripod(grp, dur)
            elif act in ("alttripod", "alt_tripod"):
                cnt = int(parts[1]) if len(parts) > 1 else 4
                toggle_tripod(cnt)
            elif act == "flat":
                flat_pose()
            elif act == "wakeup":
                wakeup()
            elif act == "sleep":
                sleep()
            elif act == "speed" and len(parts) > 1:
                speed(float(parts[1]))
            elif act == "pushups":
                count = int(parts[1]) if len(parts) > 1 else 3
                pushups(count)
            elif act == "twist":
                body_twist()
            elif act == "testall":
                test_all_sequential()
            elif act == "wave":
                wave_all()
            elif act == "testleg" and len(parts) > 1:
                test_leg(int(parts[1]))
            elif act == "servo" and len(parts) >= 4:
                dur = int(parts[4]) if len(parts) >= 5 else 250
                servo(int(parts[1]), parts[2], float(parts[3]), dur)
            elif act in ("rawall", "raw_all") and len(parts) >= 4:
                dur = int(parts[4]) if len(parts) >= 5 else 350
                raw_all(float(parts[1]), float(parts[2]), float(parts[3]), dur)
            elif act in ("offsets", "table"):
                print_offsets()
            elif act == "ch" and len(parts) >= 3:
                test_channel(int(parts[1]), int(parts[2]))
            elif act == "leg" and len(parts) >= 5:
                dur = int(parts[5]) if len(parts) >= 6 else DEFAULT_MOVE_MS
                leg_angles(int(parts[1]), float(parts[2]), float(parts[3]), float(parts[4]), dur)
            elif act == "foot" and len(parts) >= 5:
                dur = int(parts[5]) if len(parts) >= 6 else DEFAULT_MOVE_MS
                leg_foot(int(parts[1]), float(parts[2]), float(parts[3]), float(parts[4]), dur)
            elif act == "offset" and len(parts) >= 4:
                set_offset(int(parts[1]), int(parts[2]), float(parts[3]))
            elif act in ("exit", "quit"):
                break
            else:
                print("Unknown command. Type 'help'.")
        except Exception as e:
            print("Error:", e)


# ─────────────────────────────────────────────
#  BOOT SEQUENCE
# ─────────────────────────────────────────────

print("\n════════════════════════════════════════════════════════")
print("  HexaPud — 6-Leg Calibration & Electronics Test Ready")
print("════════════════════════════════════════════════════════")

# Stagger initial boot center: 100ms per leg so power supply doesn't get 18x inrush spike!
print("Staggered boot initialization (bringing each leg to 90°)...")
for l_idx in range(6):
    for j_idx in range(3):
        write_leg_joint(l_idx, j_idx, 90.0)
    time.sleep_ms(80)

print("All legs initialized.\n")

if __name__ == "__main__":
    cli()
