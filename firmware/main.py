"""
HexaPud - 18-DOF Hexapod Robot Controller
MicroPython firmware for ESP32 with dual PCA9685 PWM drivers.

Features:
- Analytical 3-DOF geometric inverse kinematics per leg
- 8-way omnidirectional tripod gait locomotion
- Dual-mode morphing (6-leg hexapod & 4-leg quadruped crawl)
- Real-time web cockpit hosted over standalone Wi-Fi SoftAP
- Non-blocking socket polling for low-latency emergency stop
"""

import network
import socket
import time
import math
from machine import Pin, I2C
from pca9685 import PCA9685

try:
    import uselect as select
except ImportError:
    import select

# --- Wi-Fi Configuration ---
WIFI_SSID = "HexaPud-Bot"
WIFI_PASS = "12345678"

# --- Hardware Configuration ---
I2C_SDA = 21
I2C_SCL = 23
PCA1_ADDR = 0x40
PCA2_ADDR = 0x41

SERVO_MIN_US = 500
SERVO_MAX_US = 2500

UPDATE_MS = 20
DEFAULT_MOVE_MS = 220
SPEED_SCALE = 1.6

# Hardware safety limits (degrees)
JOINT_LIMITS = [
    (50.0, 130.0),  # Coxa  (Joint 0)
    (30.0, 150.0),  # Femur (Joint 1)
    (30.0, 150.0),  # Tibia (Joint 2)
]

# Leg mounting geometry and calibration offsets
LEG_CONFIGS = [
    {
        "name": "Front-Left (White PLA)",
        "board": 1, "ch_coxa": 0, "ch_femur": 1, "ch_tibia": 2,
        "mount_angle": 30.0, "dir": [1, 1, 1],
        "offset": [5.0, 10.0, -3.0]
    },
    {
        "name": "Middle-Left (Black PLA)",
        "board": 1, "ch_coxa": 4, "ch_femur": 5, "ch_tibia": 6,
        "mount_angle": 90.0, "dir": [1, 1, 1],
        "offset": [0.0, -2.0, -21.0]
    },
    {
        "name": "Back-Left (Black PLA)",
        "board": 1, "ch_coxa": 8, "ch_femur": 9, "ch_tibia": 10,
        "mount_angle": 150.0, "dir": [1, 1, 1],
        "offset": [0.0, 5.0, -15.0]
    },
    {
        "name": "Back-Right (Black PLA)",
        "board": 2, "ch_coxa": 0, "ch_femur": 1, "ch_tibia": 2,
        "mount_angle": -150.0, "dir": [-1, -1, -1],
        "offset": [0.0, -14.0, 3.0]
    },
    {
        "name": "Middle-Right (Black PLA)",
        "board": 2, "ch_coxa": 4, "ch_femur": 5, "ch_tibia": 6,
        "mount_angle": -90.0, "dir": [-1, -1, -1],
        "offset": [-8.0, -5.0, 13.0]
    },
    {
        "name": "Front-Right (Black PLA)",
        "board": 2, "ch_coxa": 8, "ch_femur": 9, "ch_tibia": 10,
        "mount_angle": -30.0, "dir": [-1, -1, -1],
        "offset": [-7.0, -7.0, 4.0]
    }
]

# Physical link lengths (mm)
COXA_LEN = 51.0
FEMUR_LEN = 80.0
TIBIA_LEN = 128.0

# Neutral standing coordinates (mm)
STANDING_X = 135.0
STANDING_Z = -140.0
height_offset_z = 0.0

current_angles = [[90.0, 90.0, 90.0] for _ in range(6)]
emergency_stop_flag = False
current_robot_mode = "hexapod"

# --- I2C & PCA9685 Driver Initialization ---
time.sleep_ms(100)
i2c = I2C(0, scl=Pin(I2C_SCL), sda=Pin(I2C_SDA), freq=400000)
detected = i2c.scan()
print("[I2C] Detected devices:", [hex(d) for d in detected])

pwm1 = PCA9685(i2c, address=PCA1_ADDR) if PCA1_ADDR in detected else None
pwm2 = PCA9685(i2c, address=PCA2_ADDR) if PCA2_ADDR in detected else None
if pwm1: pwm1.set_pwm_freq(50)
if pwm2: pwm2.set_pwm_freq(50)


def clamp(val, min_v, max_v):
    return max(min_v, min(max_v, val))


def ease_in_out(t):
    t = clamp(t, 0.0, 1.0)
    return 0.5 * (1.0 - math.cos(math.pi * t))


def _write_raw_hw(board_id, channel, angle_deg):
    angle = clamp(angle_deg, 10.0, 170.0)
    pulse_us = SERVO_MIN_US + (angle / 180.0) * (SERVO_MAX_US - SERVO_MIN_US)
    if board_id == 1 and pwm1:
        pwm1.set_servo_pulse(channel, pulse_us)
    elif board_id == 2 and pwm2:
        pwm2.set_servo_pulse(channel, pulse_us)


def write_leg_joint(leg_idx, joint_idx, logical_angle):
    cfg = LEG_CONFIGS[leg_idx]
    d = cfg["dir"][joint_idx]
    off = cfg["offset"][joint_idx]
    ch = [cfg["ch_coxa"], cfg["ch_femur"], cfg["ch_tibia"]][joint_idx]

    hw_angle = (180.0 - logical_angle) if d == -1 else logical_angle
    hw_angle += off

    min_lim, max_lim = JOINT_LIMITS[joint_idx]
    hw_angle = clamp(hw_angle, min_lim, max_lim)

    current_angles[leg_idx][joint_idx] = logical_angle
    _write_raw_hw(cfg["board"], ch, hw_angle)


# --- Inverse Kinematics Engine ---
def solve_ik(x, y, z):
    """
    3-DOF analytical geometric inverse kinematics.
    Returns joint angles for coxa, femur, and tibia.
    """
    res = {'coxa': 0.0, 'femur': 0.0, 'tibia': 0.0, 'valid': False}
    res['coxa'] = math.atan2(y, x) * 180.0 / math.pi

    horiz_dist = math.sqrt(x * x + y * y)
    d_horiz = horiz_dist - COXA_LEN
    if d_horiz < 0:
        return res

    D = math.sqrt(d_horiz * d_horiz + z * z)
    reach_max = FEMUR_LEN + TIBIA_LEN
    reach_min = abs(FEMUR_LEN - TIBIA_LEN)
    if D > reach_max or D < reach_min:
        return res

    alpha = math.atan2(-z, d_horiz)
    cos_b = clamp((FEMUR_LEN**2 + D**2 - TIBIA_LEN**2) / (2.0 * FEMUR_LEN * D), -1.0, 1.0)
    beta = math.acos(cos_b)
    res['femur'] = (alpha - beta) * 180.0 / math.pi

    cos_g = clamp((FEMUR_LEN**2 + TIBIA_LEN**2 - D**2) / (2.0 * FEMUR_LEN * TIBIA_LEN), -1.0, 1.0)
    res['tibia'] = math.acos(cos_g) * 180.0 / math.pi
    res['valid'] = True
    return res


def ik_to_servo(ik_res):
    return [90.0 + ik_res['coxa'], 90.0 + ik_res['femur'], ik_res['tibia']]


# --- Motion Engine ---
def move_all_smooth(target_all_angles, duration_ms=DEFAULT_MOVE_MS):
    global emergency_stop_flag
    duration_ms = max(35, int(duration_ms / SPEED_SCALE))
    start_angles = [[current_angles[l][j] for j in range(3)] for l in range(6)]
    steps = max(1, int(duration_ms // UPDATE_MS))

    for s_idx in range(1, steps + 1):
        if emergency_stop_flag or check_network_emergency():
            break
        s = ease_in_out(s_idx / steps)
        for l in range(6):
            for j in range(3):
                interp = start_angles[l][j] + (target_all_angles[l][j] - start_angles[l][j]) * s
                write_leg_joint(l, j, interp)
        time.sleep_ms(UPDATE_MS)


def play_keyframes(keyframe_list):
    global emergency_stop_flag
    for name, pose, dur in keyframe_list:
        if emergency_stop_flag or check_network_emergency():
            break
        target_6x3 = [list(pose) for _ in range(6)] if len(pose) == 3 else pose
        move_all_smooth(target_6x3, dur)
        time.sleep_ms(15)


def stand(duration_ms=DEFAULT_MOVE_MS):
    ik = solve_ik(STANDING_X, 0, STANDING_Z + height_offset_z)
    ang = ik_to_servo(ik) if ik['valid'] else [90.0, 99.0, 99.0]
    move_all_smooth([list(ang) for _ in range(6)], duration_ms)


def flat_pose(duration_ms=DEFAULT_MOVE_MS):
    move_all_smooth([[90.0, 90.0, 30.0] for _ in range(6)], duration_ms)


def wakeup():
    ik_stand = solve_ik(STANDING_X, 0, STANDING_Z + height_offset_z)
    stand_angles = ik_to_servo(ik_stand) if ik_stand['valid'] else [90.0, 99.0, 99.0]
    sequence = [
        ("flat",      [90.0,  90.0,  30.0],  180),
        ("knees_up",  [90.0, 145.0,  30.0],  220),
        ("tibias_dn", [90.0, 145.0, 125.0],  220),
        ("stand",     stand_angles,          260)
    ]
    play_keyframes(sequence)


def sleep():
    sequence = [
        ("unload", [90.0, 130.0, 110.0], 180),
        ("extend", [90.0, 120.0,  40.0], 200),
        ("lower",  [90.0,  95.0,  30.0], 200),
        ("relax",  [90.0,  90.0,  30.0], 150)
    ]
    play_keyframes(sequence)


def pushups(count=3):
    z_low = STANDING_Z + height_offset_z + 55.0
    z_high = STANDING_Z + height_offset_z - 35.0
    ik_l = solve_ik(STANDING_X + 20.0, 0, z_low)
    ik_h = solve_ik(STANDING_X - 10.0, 0, z_high)

    if ik_l['valid'] and ik_h['valid']:
        l_ang = ik_to_servo(ik_l)
        h_ang = ik_to_servo(ik_h)
        for _ in range(count):
            if emergency_stop_flag or check_network_emergency():
                break
            move_all_smooth([l_ang for _ in range(6)], 190)
            time.sleep_ms(30)
            move_all_smooth([h_ang for _ in range(6)], 160)
            time.sleep_ms(30)

    if current_robot_mode == "quadruped":
        quadruped_stand()
    else:
        stand(160)


def body_twist(cycles=2):
    """Rotates chassis yaw while keeping feet planted on the floor."""
    global emergency_stop_flag
    phase = 0.0
    dt = UPDATE_MS / 1000.0
    cycle_time = 1.5
    total_steps = int((cycles * cycle_time) / dt)
    max_yaw_deg = 24.0

    print("[Motion] Body twist started")
    for _ in range(total_steps):
        if emergency_stop_flag or check_network_emergency():
            break

        phase += 2.0 * math.pi * (dt / cycle_time)
        current_yaw = max_yaw_deg * math.sin(phase)
        yaw_rad = math.radians(current_yaw)

        targets = []
        for l in range(6):
            is_right = (l in (3, 4, 5))
            r_total = 235.0
            r_chassis = 100.0
            lx = r_total * math.cos(yaw_rad) - r_chassis
            raw_ly = -r_total * math.sin(yaw_rad)

            ly = -raw_ly if is_right else raw_ly
            lz = STANDING_Z + height_offset_z

            ik = solve_ik(lx, ly, lz)
            targets.append(ik_to_servo(ik) if ik['valid'] else current_angles[l])

        for l in range(6):
            for j in range(3):
                write_leg_joint(l, j, targets[l][j])
        time.sleep_ms(UPDATE_MS)

    if current_robot_mode == "quadruped":
        quadruped_stand()
    else:
        stand(160)


def wave_ripple(cycles=3):
    global emergency_stop_flag
    phase = 0.0
    dt = UPDATE_MS / 1000.0
    cycle_time = 1.3
    total_steps = int((cycles * cycle_time) / dt)

    for _ in range(total_steps):
        if emergency_stop_flag or check_network_emergency():
            break
        phase += 2.0 * math.pi * (dt / cycle_time)
        for l in range(6):
            leg_phase = phase + (l * math.pi / 3.0)
            x = STANDING_X
            y = 30.0 * math.sin(leg_phase)
            z = STANDING_Z + height_offset_z + 38.0 * math.cos(leg_phase)
            ik = solve_ik(x, y, z)
            if ik['valid']:
                s = ik_to_servo(ik)
                for j in range(3):
                    write_leg_joint(l, j, s[j])
        time.sleep_ms(UPDATE_MS)

    if current_robot_mode == "quadruped":
        quadruped_stand()
    else:
        stand(160)


def tripod_high(group='a'):
    global emergency_stop_flag
    standing = [0, 2, 4] if group == 'a' else [1, 3, 5]
    ik_stand = solve_ik(STANDING_X, 0, STANDING_Z + height_offset_z)
    s_ang = ik_to_servo(ik_stand) if ik_stand['valid'] else [90.0, 99.0, 99.0]
    high_lift_ang = [90.0, 148.0, 35.0]

    targets = [list(s_ang) if i in standing else list(high_lift_ang) for i in range(6)]
    move_all_smooth(targets, 260)


def quadruped_stand():
    """Tucks middle legs into mantis pose; corner legs form stable rectangle."""
    global emergency_stop_flag
    ik_quad = solve_ik(145.0, 0, STANDING_Z + height_offset_z)
    q_ang = ik_to_servo(ik_quad) if ik_quad['valid'] else [90.0, 99.0, 99.0]
    mantis_high_ang = [90.0, 148.0, 35.0]

    targets = []
    for l in range(6):
        if l in (1, 4):
            targets.append(list(mantis_high_ang))
        else:
            targets.append(list(q_ang))

    move_all_smooth(targets, 280)


def quadruped_walk(direction_key="f", duration_sec=2.5):
    """4-leg diagonal crawl gait with middle legs locked in mantis stance."""
    global emergency_stop_flag
    step_len = 54.0
    step_lift = 42.0
    diag = step_len * 0.7071

    dirs = {
        "f":  ( step_len,  0.0),
        "b":  (-step_len,  0.0),
        "l":  ( 0.0,       step_len),
        "r":  ( 0.0,      -step_len),
        "fl": ( diag,      diag),
        "fr": ( diag,     -diag),
        "bl": (-diag,      diag),
        "br": (-diag,     -diag),
    }

    world_dx, world_dy = dirs.get(direction_key, (step_len, 0.0))
    mantis_high_ang = [90.0, 148.0, 35.0]

    base_cycle = 0.68
    cycle_time_sec = max(0.42, base_cycle / (SPEED_SCALE / 1.6))
    gait_speed = (UPDATE_MS / 1000.0) / cycle_time_sec
    gait_phase = 0.0
    total_ticks = int((duration_sec * 1000.0) / UPDATE_MS)

    for _ in range(total_ticks):
        if emergency_stop_flag or check_network_emergency():
            break

        gait_phase = (gait_phase + gait_speed) % 1.0

        for l in range(6):
            if l in (1, 4):
                for j in range(3):
                    write_leg_joint(l, j, mantis_high_ang[j])
                continue

            is_pair_a = (l in (0, 3))
            leg_p = gait_phase if is_pair_a else ((gait_phase + 0.5) % 1.0)
            m_rad = math.radians(LEG_CONFIGS[l]["mount_angle"])
            is_right = (l in (3, 4, 5))

            raw_lx = world_dx * math.cos(m_rad) + world_dy * math.sin(m_rad)
            raw_ly = -world_dx * math.sin(m_rad) + world_dy * math.cos(m_rad)

            local_dir_x = raw_lx * 0.5
            local_dir_y = (-raw_ly if is_right else raw_ly) * 0.5

            if leg_p < 0.45:
                t = leg_p / 0.45
                s = ease_in_out(t)
                lz = STANDING_Z + height_offset_z + step_lift * math.sin(math.pi * t)
                p = -1.0 + 2.0 * s
            else:
                t = (leg_p - 0.45) / 0.55
                s = ease_in_out(t)
                lz = STANDING_Z + height_offset_z
                p = 1.0 - 2.0 * s

            lx = 145.0 + p * local_dir_x
            ly = p * local_dir_y

            ik = solve_ik(lx, ly, lz)
            if ik['valid']:
                s_angles = ik_to_servo(ik)
                for j in range(3):
                    write_leg_joint(l, j, s_angles[j])

        time.sleep_ms(UPDATE_MS)

    quadruped_stand()


def rotate_step(direction="left", cycles=3):
    """Turns the robot on the spot using coordinated tripod yaw steps."""
    global emergency_stop_flag
    yaw_sign = 1.0 if direction == "left" else -1.0
    tangential_stride = yaw_sign * 46.0
    step_h = 44.0

    for _ in range(cycles):
        if emergency_stop_flag or check_network_emergency():
            break

        for trip_idx in (0, 1):
            if emergency_stop_flag or check_network_emergency():
                break
            swing_legs = [0, 2, 4] if trip_idx == 0 else [1, 3, 5]
            stance_legs = [1, 3, 5] if trip_idx == 0 else [0, 2, 4]

            target_lift = [list(current_angles[i]) for i in range(6)]
            for l in swing_legs:
                is_right = (l in (3, 4, 5))
                raw_ly = tangential_stride * 0.5
                ly = -raw_ly if is_right else raw_ly
                ik = solve_ik(STANDING_X, ly, STANDING_Z + height_offset_z + step_h)
                if ik['valid']:
                    target_lift[l] = ik_to_servo(ik)
            move_all_smooth(target_lift, 95)

            target_step = [list(current_angles[i]) for i in range(6)]
            for l in swing_legs:
                is_right = (l in (3, 4, 5))
                raw_ly = tangential_stride * 0.5
                ly = -raw_ly if is_right else raw_ly
                ik = solve_ik(STANDING_X, ly, STANDING_Z + height_offset_z)
                if ik['valid']:
                    target_step[l] = ik_to_servo(ik)
            for l in stance_legs:
                is_right = (l in (3, 4, 5))
                raw_ly = -tangential_stride * 0.5
                ly = -raw_ly if is_right else raw_ly
                ik = solve_ik(STANDING_X, ly, STANDING_Z + height_offset_z)
                if ik['valid']:
                    target_step[l] = ik_to_servo(ik)
            move_all_smooth(target_step, 135)

    if current_robot_mode == "quadruped":
        quadruped_stand()
    else:
        stand(150)


def walk_burst(direction_key, duration_sec):
    """
    Executes an 8-way tripod walk burst for the specified duration.
    Coordinates world direction vector across 6 leg frames.
    """
    global emergency_stop_flag
    step_len = 65.0
    step_lift = 46.0
    diag = step_len * 0.7071

    dirs = {
        "f":  ( step_len,  0.0),
        "b":  (-step_len,  0.0),
        "l":  ( 0.0,       step_len),
        "r":  ( 0.0,      -step_len),
        "fl": ( diag,      diag),
        "fr": ( diag,     -diag),
        "bl": (-diag,      diag),
        "br": (-diag,     -diag),
    }

    world_dx, world_dy = dirs.get(direction_key, (step_len, 0.0))

    base_cycle = 0.62
    cycle_time_sec = max(0.38, base_cycle / (SPEED_SCALE / 1.6))
    gait_speed = (UPDATE_MS / 1000.0) / cycle_time_sec
    gait_phase = 0.0
    total_ticks = int((duration_sec * 1000.0) / UPDATE_MS)

    print("[Motion] Walk burst:", direction_key.upper(), "for", duration_sec, "s")

    for _ in range(total_ticks):
        if check_network_emergency() or emergency_stop_flag:
            print("[Motion] Aborted by emergency stop")
            break

        gait_phase = (gait_phase + gait_speed) % 1.0

        for l in range(6):
            is_tripod_a = (l in (0, 2, 4))
            leg_p = gait_phase if is_tripod_a else ((gait_phase + 0.5) % 1.0)
            m_rad = math.radians(LEG_CONFIGS[l]["mount_angle"])
            is_right_side = (l in (3, 4, 5))

            raw_lx = world_dx * math.cos(m_rad) + world_dy * math.sin(m_rad)
            raw_ly = -world_dx * math.sin(m_rad) + world_dy * math.cos(m_rad)

            local_dir_x = raw_lx * 0.5
            local_dir_y = (-raw_ly if is_right_side else raw_ly) * 0.5

            if leg_p < 0.5:
                t = leg_p / 0.5
                s = ease_in_out(t)
                lz = STANDING_Z + height_offset_z + step_lift * math.sin(math.pi * t)
                p = -1.0 + 2.0 * s
            else:
                t = (leg_p - 0.5) / 0.5
                s = ease_in_out(t)
                lz = STANDING_Z + height_offset_z
                p = 1.0 - 2.0 * s

            lx = STANDING_X + p * local_dir_x
            ly = p * local_dir_y

            ik = solve_ik(lx, ly, lz)
            if ik['valid']:
                s_angles = ik_to_servo(ik)
                for j in range(3):
                    write_leg_joint(l, j, s_angles[j])

        time.sleep_ms(UPDATE_MS)

    stand(140)


def check_network_emergency():
    """Non-blocking socket check for incoming emergency stop command."""
    global emergency_stop_flag
    try:
        events = poller.poll(0)
        if not events:
            return False
        conn, client = s.accept()
        conn.settimeout(0.04)
        try:
            req = conn.recv(256).decode('utf-8', 'ignore')
        except:
            req = ""

        if req and ("a=stop" in req or "action=stop" in req):
            print("[Emergency] Stop received")
            emergency_stop_flag = True
            try:
                conn.sendall(b"HTTP/1.1 204 No Content\r\nAccess-Control-Allow-Origin: *\r\nConnection: close\r\n\r\n")
            except:
                pass
            conn.close()
            return True
        conn.close()
    except Exception:
        pass
    return False


# --- Embedded Web Cockpit ---
HTML_PAGE = """<!DOCTYPE html>
<html lang="en">
<head>
<meta charset="utf-8">
<meta name="viewport" content="width=device-width,initial-scale=1,user-scalable=no">
<title>HexaPud Controller</title>
<style>
*{box-sizing:border-box;margin:0;padding:0;font-family:-apple-system,BlinkMacSystemFont,"Segoe UI",Roboto,sans-serif;-webkit-tap-highlight-color:transparent}
body{background:#0a0a0a;color:#ededed;padding:16px 12px;display:flex;flex-direction:column;align-items:center;min-height:100vh}
.c{width:100%;max-width:400px;display:flex;flex-direction:column;gap:14px}
.header{display:flex;align-items:center;justify-content:space-between;padding:4px 2px}
.logo{font-size:1.1rem;font-weight:700;letter-spacing:2px;color:#ffffff}
.badge{font-size:0.65rem;background:#1e293b;border:1px solid #334155;color:#38bdf8;padding:2px 6px;border-radius:4px;font-weight:600;margin-left:6px}
#st{font-family:ui-monospace,monospace;font-size:0.75rem;color:#888888;letter-spacing:0.5px}
.card{background:#141414;border:1px solid #262626;border-radius:12px;padding:14px}
.title{font-size:0.7rem;font-weight:600;letter-spacing:1px;text-transform:uppercase;color:#737373;margin-bottom:12px}
.btn{background:#1c1c1c;border:1px solid #2e2e2e;color:#e5e5e5;padding:12px 6px;border-radius:8px;font-weight:600;font-size:0.8rem;letter-spacing:0.5px;cursor:pointer;display:flex;align-items:center;justify-content:center;text-align:center;min-height:44px;width:100%}
.btn:active{background:#ffffff;color:#000000}
.btn-primary{background:#ededed;color:#0a0a0a;border-color:#ededed}
.btn-danger{background:#3a1414;border-color:#5c1d1d;color:#ef4444;font-weight:700}
.btn-danger:active{background:#ef4444;color:#ffffff}
.btn-dim{background:#141414;border-color:#262626;color:#a3a3a3}
.mode-toggle{display:grid;grid-template-columns:1fr 1fr;background:#0d0d0d;border:1px solid #262626;border-radius:10px;padding:3px;gap:4px}
.mode-btn{background:transparent;border:none;color:#737373;padding:11px;border-radius:7px;font-size:0.75rem;font-weight:700;letter-spacing:1.5px;cursor:pointer;transition:all 0.15s ease;text-align:center}
.mode-btn.active{background:#ededed;color:#0a0a0a;box-shadow:0 2px 6px rgba(0,0,0,0.5)}
.g2{display:grid;grid-template-columns:repeat(2,1fr);gap:8px}
.g3{display:grid;grid-template-columns:repeat(3,1fr);gap:8px}
.row{display:flex;justify-content:space-between;font-size:0.75rem;color:#a3a3a3;margin-bottom:8px}
input[type=range]{width:100%;accent-color:#ffffff;background:#262626;height:4px;border-radius:2px;outline:none}
.joy-wrap{display:flex;flex-direction:column;align-items:center;justify-content:center;padding:8px 0}
#joy-zone{width:210px;height:210px;background:#111111;border:1px solid #333333;border-radius:50%;position:relative;touch-action:none}
#joy-thumb{width:56px;height:56px;background:#ededed;border-radius:50%;position:absolute;top:50%;left:50%;transform:translate(-50%,-50%);pointer-events:none;box-shadow:0 2px 8px rgba(0,0,0,0.6);transition:transform 0.08s ease}
.spoke{position:absolute;width:100%;height:1px;background:#222222;top:50%;left:0}
.spoke-45{transform:rotate(45deg)}
.spoke-90{transform:rotate(90deg)}
.spoke-135{transform:rotate(135deg)}
.center-deadband{position:absolute;width:60px;height:60px;border:1px dashed #282828;border-radius:50%;top:50%;left:50%;transform:translate(-50%,-50%)}
#joy-val{font-family:ui-monospace,monospace;font-size:0.7rem;color:#737373;margin-top:10px}
#lock-badge{display:none;background:#2b2205;border:1px solid #854d0e;color:#facc15;padding:4px 8px;border-radius:6px;font-size:0.75rem;font-weight:600;margin-top:6px;text-align:center}
</style>
</head>
<body>
<div class="c">
  <div class="header">
    <div style="display:flex;align-items:center">
      <div class="logo">HEXAPUD</div>
      <span class="badge">V2.0</span>
    </div>
    <div id="st">READY</div>
  </div>

  <div class="mode-toggle">
    <button id="btn-mode-hex" class="mode-btn active" onclick="switchMode('hexapod')">6-LEG HEXAPOD</button>
    <button id="btn-mode-quad" class="mode-btn" onclick="switchMode('quadruped')">4-LEG QUADRUPED</button>
  </div>

  <button class="btn btn-danger" style="padding:15px;letter-spacing:1.5px;font-size:0.95rem" onclick="run('stop')">
    STOP &bull; EMERGENCY HALT
  </button>

  <div class="card">
    <div class="title" style="text-align:center">8-Way Stride Controller</div>
    <div class="joy-wrap">
      <div id="joy-zone">
        <div class="spoke"></div>
        <div class="spoke spoke-45"></div>
        <div class="spoke spoke-90"></div>
        <div class="spoke spoke-135"></div>
        <div class="center-deadband"></div>
        <div id="joy-thumb"></div>
      </div>
      <div id="joy-val">PULL &amp; RELEASE TO DISPATCH</div>
      <div id="lock-badge">EXECUTING STRIDE... INPUT LOCKED</div>
    </div>
    <div class="g2" style="margin-top:8px">
      <button class="btn" onclick="run('turn_left')">ROTATE LEFT</button>
      <button class="btn" onclick="run('turn_right')">ROTATE RIGHT</button>
    </div>
  </div>

  <div class="card">
    <div class="title">Locomotion Speed</div>
    <div class="g3">
      <button id="spd-norm" class="btn" onclick="setSpeed(1.2,'spd-norm')">NORMAL</button>
      <button id="spd-fast" class="btn btn-primary" onclick="setSpeed(1.8,'spd-fast')">FAST</button>
      <button id="spd-turbo" class="btn" onclick="setSpeed(2.4,'spd-turbo')">TURBO</button>
    </div>
  </div>

  <div class="card">
    <div class="title">Stance &amp; Resting</div>
    <div class="g3">
      <button class="btn btn-primary" onclick="run('stand')">STAND</button>
      <button class="btn" onclick="run('flat')">FLAT</button>
      <button class="btn btn-dim" onclick="run('sleep')">SLEEP</button>
    </div>
  </div>

  <div class="card">
    <div class="title">Routines &amp; Poses</div>
    <div class="g3">
      <button class="btn" onclick="run('tripod_a')">TRIPOD A</button>
      <button class="btn" onclick="run('tripod_b')">TRIPOD B</button>
      <button class="btn" onclick="run('wakeup')">WAKE UP</button>
      <button class="btn" onclick="run('pushups')">PUSH-UPS</button>
      <button class="btn" onclick="run('twist')">TWIST (24&deg;)</button>
      <button class="btn" onclick="run('wave')">WAVE</button>
    </div>
  </div>

  <div class="card">
    <div class="row"><span>CHASSIS ELEVATION</span><span id="hv">0 mm</span></div>
    <input type="range" id="h-slider" min="-65" max="65" value="0" oninput="updateHDisplay(this.value)" onchange="sendH(this.value)">
  </div>
</div>

<script>
var currentMode = 'hexapod';
var isLocked = false;
var lockTimer = null;

function switchMode(m) {
  setLockState(false, 0);
  currentMode = m;
  document.getElementById('btn-mode-hex').className = 'mode-btn' + (m === 'hexapod' ? ' active' : '');
  document.getElementById('btn-mode-quad').className = 'mode-btn' + (m === 'quadruped' ? ' active' : '');
  var el = document.getElementById('st');
  el.style.color = '#ededed';
  el.innerText = (m === 'hexapod' ? 'HEXAPOD (6-LEG)' : 'QUADRUPED (4-LEG)');
  var img = new Image();
  img.src = '/api?m=' + m + '&t=' + Date.now();
}

function setSpeed(spd, btnId){
  if(isLocked) return;
  ['spd-norm','spd-fast','spd-turbo'].forEach(function(id){
    document.getElementById(id).className = 'btn' + (id === btnId ? ' btn-primary' : '');
  });
  var el = document.getElementById('st');
  el.style.color = '#ededed';
  el.innerText = 'SPEED: ' + (spd === 1.2 ? 'NORMAL' : (spd === 1.8 ? 'FAST' : 'TURBO'));
  var img = new Image();
  img.src = '/api?spd=' + spd + '&t=' + Date.now();
}

function setLockState(locked, durationMs) {
  isLocked = locked;
  var badge = document.getElementById('lock-badge');
  badge.style.display = locked ? 'block' : 'none';
  if(locked) {
    clearTimeout(lockTimer);
    lockTimer = setTimeout(function(){ setLockState(false, 0); }, durationMs + 100);
  }
}

function run(act){
  if(act === 'stop') {
    setLockState(false, 0);
    try { navigator.sendBeacon('/api?a=stop&t=' + Date.now()); } catch(e){}
    var img = new Image();
    img.src = '/api?a=stop&t=' + Date.now();
    var el = document.getElementById('st');
    el.style.color = '#ef4444';
    el.innerText = 'EMERGENCY STOP';
    return;
  }
  if(isLocked) return;
  var el = document.getElementById('st');
  el.style.color = '#ededed';
  el.innerText = act.toUpperCase();
  var img = new Image();
  img.src = '/api?a=' + encodeURIComponent(act) + '&t=' + Date.now();
}

function updateHDisplay(v){
  document.getElementById('hv').innerText = (v > 0 ? '+' : '') + v + ' mm';
}

function sendH(v){
  if(isLocked) return;
  updateHDisplay(v);
  var img = new Image();
  img.src = '/api?h=' + encodeURIComponent(v) + '&t=' + Date.now();
}

var zone = document.getElementById('joy-zone');
var thumb = document.getElementById('joy-thumb');
var joyVal = document.getElementById('joy-val');
var radius = 72;
var active = false;
var latchedDir = null;
var latchedDuration = 0.0;

var DIR_NAMES = {
  "f": "FORWARD", "b": "BACKWARD", "l": "LEFT", "r": "RIGHT",
  "fl": "FORWARD-LEFT", "fr": "FORWARD-RIGHT", "bl": "BACKWARD-LEFT", "br": "BACKWARD-RIGHT"
};

function handleMove(clientX, clientY){
  if(isLocked) return;
  var rect = zone.getBoundingClientRect();
  var cx = rect.left + rect.width / 2;
  var cy = rect.top + rect.height / 2;
  var dx = clientX - cx;
  var dy = clientY - cy;
  var dist = Math.sqrt(dx * dx + dy * dy);

  if(dist > radius){
    dx = (dx / dist) * radius;
    dy = (dy / dist) * radius;
    dist = radius;
  }

  thumb.style.transform = 'translate(' + (dx - 28) + 'px,' + (dy - 28) + 'px)';

  if(dist < 22){
    latchedDir = null;
    latchedDuration = 0.0;
    joyVal.innerText = 'PULL & RELEASE TO DISPATCH';
    return;
  }

  var deg = (Math.atan2(dx, -dy) * 180 / Math.PI + 360) % 360;
  var sector = Math.round(deg / 45) % 8;
  var keyMap = ["f", "fr", "r", "br", "b", "bl", "l", "fl"];
  latchedDir = keyMap[sector];

  var magFrac = (dist - 22) / (radius - 22);
  latchedDuration = parseFloat((0.8 + magFrac * 2.2).toFixed(1));
  joyVal.innerText = DIR_NAMES[latchedDir] + ' (' + latchedDuration + 's)';
}

function handleRelease(){
  thumb.style.transform = 'translate(-50%,-50%)';
  if(isLocked || !latchedDir || latchedDuration <= 0.0){
    joyVal.innerText = 'PULL & RELEASE TO DISPATCH';
    return;
  }

  var durMs = Math.round(latchedDuration * 1000);
  setLockState(true, durMs);
  joyVal.innerText = 'EXECUTING: ' + DIR_NAMES[latchedDir] + ' (' + latchedDuration + 's)';

  var img = new Image();
  img.src = '/api?w=' + latchedDir + '&dur=' + latchedDuration + '&m=' + currentMode + '&t=' + Date.now();

  latchedDir = null;
  latchedDuration = 0.0;
}

zone.addEventListener('pointerdown', function(e){
  if(isLocked) return;
  active = true;
  zone.setPointerCapture(e.pointerId);
  handleMove(e.clientX, e.clientY);
});
zone.addEventListener('pointermove', function(e){
  if(active) handleMove(e.clientX, e.clientY);
});
zone.addEventListener('pointerup', function(e){
  active = false;
  handleRelease();
});
zone.addEventListener('pointercancel', function(e){
  active = false;
  handleRelease();
});
</script>
</body>
</html>"""

# --- Wi-Fi SoftAP Setup ---
print("[Network] Initializing Wi-Fi SoftAP...")
ap = network.WLAN(network.AP_IF)
ap.active(True)
ap.config(essid=WIFI_SSID, password=WIFI_PASS, authmode=network.AUTH_WPA_WPA2_PSK if WIFI_PASS else network.AUTH_OPEN)

while not ap.active():
    time.sleep_ms(100)

print("[Network] SoftAP online: SSID =", WIFI_SSID, "| IP = 192.168.4.1")

# Center all joints on startup
for l_idx in range(6):
    for j_idx in range(3):
        write_leg_joint(l_idx, j_idx, 90.0)
    time.sleep_ms(40)

# --- HTTP Server & Dispatch Loop ---
s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
s.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
s.bind(('', 80))
s.listen(3)
s.settimeout(0.1)

poller = select.poll()
poller.register(s, select.POLLIN)

print("[Server] Listening on port 80\n")

while True:
    try:
        conn, client = s.accept()
    except OSError:
        time.sleep_ms(5)
        continue

    try:
        conn.settimeout(0.2)
        req = conn.recv(1024).decode('utf-8', 'ignore')
        if not req:
            conn.close()
            continue

        line = req.split('\r\n')[0]
        url = line.split()[1] if len(line.split()) > 1 else "/"

        if "w=" in url and "dur=" in url:
            try:
                w_dir = url.split("w=")[1].split("&")[0]
                w_dur = float(url.split("dur=")[1].split("&")[0])
                mode_req = current_robot_mode
                if "m=" in url:
                    mode_req = url.split("m=")[1].split("&")[0]
                    current_robot_mode = mode_req

                emergency_stop_flag = False
                conn.sendall(b"HTTP/1.1 204 No Content\r\nAccess-Control-Allow-Origin: *\r\nConnection: close\r\n\r\n")
                conn.close()

                if mode_req == "quadruped":
                    quadruped_walk(w_dir, w_dur)
                else:
                    walk_burst(w_dir, w_dur)
            except Exception as we:
                print("[Error] Walk burst failed:", we)
            continue

        elif url.startswith('/api'):
            if "m=" in url:
                new_m = url.split("m=")[1].split("&")[0]
                current_robot_mode = new_m
                print("[Mode] Switched to:", current_robot_mode)
                emergency_stop_flag = False
                if current_robot_mode == "quadruped":
                    quadruped_stand()
                else:
                    stand(200)

            elif "spd=" in url:
                spd_val = float(url.split("spd=")[1].split("&")[0].split()[0])
                SPEED_SCALE = clamp(spd_val, 0.5, 3.0)
                print("[Speed] Scale set to:", SPEED_SCALE)

            elif "a=" in url:
                act = url.split("a=")[1].split("&")[0].split()[0]
                print("[Action] Executing:", act)

                if act == "stop":
                    emergency_stop_flag = True
                    if current_robot_mode == "quadruped":
                        quadruped_stand()
                    else:
                        stand(120)
                else:
                    emergency_stop_flag = False
                    if act == "stand":
                        if current_robot_mode == "quadruped":
                            quadruped_stand()
                        else:
                            stand(200)
                    elif act in ("hex", "hexapod"):
                        current_robot_mode = "hexapod"
                        stand(200)
                    elif act in ("quad", "quadruped"):
                        current_robot_mode = "quadruped"
                        quadruped_stand()
                    elif act == "tripod_a": tripod_high('a')
                    elif act == "tripod_b": tripod_high('b')
                    elif act == "flat": flat_pose()
                    elif act == "sleep": sleep()
                    elif act == "wakeup": wakeup()
                    elif act == "pushups": pushups(3)
                    elif act == "twist": body_twist(2)
                    elif act == "wave": wave_ripple(3)
                    elif act in ("turn_left", "rotate_left"): rotate_step("left")
                    elif act in ("turn_right", "rotate_right"): rotate_step("right")

            elif "h=" in url:
                h_val = float(url.split("h=")[1].split("&")[0].split()[0])
                height_offset_z = clamp(h_val, -65.0, 65.0)
                print("[Elevation] Height offset:", height_offset_z, "mm")
                if current_robot_mode == "quadruped":
                    quadruped_stand()
                else:
                    stand(140)

            conn.sendall(b"HTTP/1.1 204 No Content\r\nAccess-Control-Allow-Origin: *\r\nConnection: close\r\n\r\n")

        elif "favicon" in url:
            conn.sendall(b"HTTP/1.1 204 No Content\r\nAccess-Control-Allow-Origin: *\r\nConnection: close\r\n\r\n")

        else:
            body = HTML_PAGE.encode('utf-8')
            hdr = "HTTP/1.1 200 OK\r\nContent-Type: text/html\r\nContent-Length: {}\r\nConnection: close\r\n\r\n".format(len(body))
            conn.sendall(hdr.encode('utf-8'))
            conn.sendall(body)

        conn.close()

    except Exception as e:
        try:
            conn.close()
        except:
            pass
