"""
PCA9685 16-Channel 12-Bit PWM Driver for MicroPython
Handles hardware I2C register configuration and servo pulse width generation.
"""

import ustruct
import time

_REG_MODE1 = 0x00
_REG_PRESCALE = 0xFE
_LED0_ON_L = 0x06

_MODE1_RESTART = 0x80
_MODE1_AI = 0x20
_MODE1_SLEEP = 0x10


class PCA9685:
    def __init__(self, i2c, address=0x40):
        self.i2c = i2c
        self.address = address
        self.reset()

    def _write_reg(self, reg, val):
        self.i2c.writeto_mem(self.address, reg, bytes([val & 0xFF]))

    def _read_reg(self, reg):
        return self.i2c.readfrom_mem(self.address, reg, 1)[0]

    def reset(self):
        self._write_reg(_REG_MODE1, _MODE1_AI)
        time.sleep_ms(10)

    def set_pwm_freq(self, freq=50):
        prescale = int(25000000.0 / (4096.0 * freq) - 1.0 + 0.5)

        old_mode = self._read_reg(_REG_MODE1)
        sleep_mode = (old_mode & 0x7F) | _MODE1_SLEEP | _MODE1_AI
        self._write_reg(_REG_MODE1, sleep_mode)
        self._write_reg(_REG_PRESCALE, prescale)

        wake_mode = (old_mode & ~_MODE1_SLEEP) | _MODE1_AI
        self._write_reg(_REG_MODE1, wake_mode)
        time.sleep_ms(5)

        self._write_reg(_REG_MODE1, wake_mode | _MODE1_RESTART)
        time.sleep_ms(5)

    def set_pwm(self, channel, on, off):
        on = max(0, min(4095, int(on)))
        off = max(0, min(4095, int(off)))
        data = ustruct.pack('<HH', on, off)
        self.i2c.writeto_mem(self.address, _LED0_ON_L + 4 * channel, data)

    def set_servo_pulse(self, channel, pulse_us, freq=50):
        period_us = 1000000.0 / freq
        ticks = int((pulse_us * 4096.0) / period_us + 0.5)
        self.set_pwm(channel, 0, ticks)
