"""
HexaPud - Single Servo Calibration Tool
CLI utility to calibrate horn offsets for individual servos.

Usage:
  1. Set BOARD_ID and CHANNEL_NO below.
  2. Run script in MicroPython (Thonny or serial REPL).
  3. Nudge the servo until mechanically aligned at 90 degrees.
  4. Type 'done' to print the calculated software offset.
"""

from machine import Pin, I2C
import time
from pca9685 import PCA9685

# --- Calibration Target ---
BOARD_ID = 2      # 1 = Left board (0x40), 2 = Right board (0x41)
CHANNEL_NO = 10   # PCA9685 channel (0 - 15)

# --- Hardware Constants ---
I2C_SDA = 21
I2C_SCL = 23
SERVO_MIN_US = 500
SERVO_MAX_US = 2500

addr = 0x40 if BOARD_ID == 1 else 0x41
i2c = I2C(0, scl=Pin(I2C_SCL), sda=Pin(I2C_SDA), freq=400000)
pwm = PCA9685(i2c, address=addr)
pwm.set_pwm_freq(50)

current_angle = 90.0


def clamp(val, min_v, max_v):
    return max(min_v, min(max_v, val))


def set_hardware_angle(deg):
    global current_angle
    current_angle = clamp(float(deg), 10.0, 170.0)
    pulse_us = SERVO_MIN_US + (current_angle / 180.0) * (SERVO_MAX_US - SERVO_MIN_US)
    pwm.set_servo_pulse(CHANNEL_NO, pulse_us)


set_hardware_angle(current_angle)

print("HexaPud Servo Calibration Utility")
print("Target: Board {}, Channel {}".format(BOARD_ID, CHANNEL_NO))
print("Current angle: {:.1f} deg".format(current_angle))
print("Commands: [+/-deg] to nudge, 'set <deg>' to jump, 'done' to compute offset, 'exit' to quit\n")

while True:
    try:
        raw_cmd = input("[Angle: {:.1f} deg] Enter command: ".format(current_angle)).strip()
    except (KeyboardInterrupt, EOFError):
        print("\nExiting calibration.")
        break

    if not raw_cmd:
        continue

    cmd = raw_cmd.lower()

    if cmd in ("exit", "quit", "q"):
        break

    elif cmd in ("done", "offset", "save"):
        offset_val = current_angle - 90.0
        print("\nCalibration Result for Channel {}:".format(CHANNEL_NO))
        print("  Raw hardware angle at neutral: {:.1f} deg".format(current_angle))
        print("  Software offset: {:+.1f} deg\n".format(offset_val))

    elif cmd.startswith("set "):
        try:
            target = float(cmd.split()[1])
            set_hardware_angle(target)
            offset_val = current_angle - 90.0
            print("  Angle set to {:.1f} deg (offset: {:+.1f} deg)".format(current_angle, offset_val))
        except (ValueError, IndexError):
            print("  Invalid syntax. Use: set <angle>")

    else:
        try:
            nudge = float(cmd)
            set_hardware_angle(current_angle + nudge)
            offset_val = current_angle - 90.0
            print("  Angle shifted to {:.1f} deg (offset: {:+.1f} deg)".format(current_angle, offset_val))
        except ValueError:
            print("  Unrecognized command. Enter number to nudge, 'set <angle>', or 'done'.")

print("Calibration session closed.")
