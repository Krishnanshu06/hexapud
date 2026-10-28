"""
PCA9685 PWM Driver for MicroPython
Follows the official NXP PCA9685 datasheet & Adafruit register sequence.
"""

import ustruct
import time

_REG_MODE1 = 0x00
_REG_PRESCALE = 0xFE
_LED0_ON_L = 0x06

# MODE1 bits
_MODE1_RESTART = 0x80
_MODE1_AI      = 0x20  # Auto-Increment enabled
_MODE1_SLEEP   = 0x10  # Low power mode / oscillator off


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
        """Reset PCA9685 into normal mode with Auto-Increment (AI) enabled."""
        self._write_reg(_REG_MODE1, _MODE1_AI)
        time.sleep_ms(10)

    def set_pwm_freq(self, freq):
        """
        Set PWM frequency in Hz (50Hz for standard servos).
        Follows NXP PCA9685 datasheet Section 7.3.1.1 restart sequence:
          1. Read MODE1
          2. Set SLEEP bit (oscillator off) while keeping AI enabled
          3. Write PRESCALE register
          4. Clear SLEEP bit
          5. Wait >= 500us for oscillator to stabilize
          6. Set RESTART bit
        """
        prescale = int(25000000.0 / (4096.0 * freq) - 1.0 + 0.5)

        old_mode = self._read_reg(_REG_MODE1)
        # Ensure SLEEP is set and RESTART is not triggered yet
        sleep_mode = (old_mode & 0x7F) | _MODE1_SLEEP | _MODE1_AI
        self._write_reg(_REG_MODE1, sleep_mode)
        self._write_reg(_REG_PRESCALE, prescale)

        # Wake up (clear SLEEP bit, keep AI enabled)
        wake_mode = (old_mode & ~_MODE1_SLEEP) | _MODE1_AI
        self._write_reg(_REG_MODE1, wake_mode)
        time.sleep_ms(5)  # Wait for oscillator (>500us required)

        # Now trigger RESTART with AI enabled
        self._write_reg(_REG_MODE1, wake_mode | _MODE1_RESTART)
        time.sleep_ms(5)

    def set_pwm(self, channel, on, off):
        """Set PWM on/off ticks for a channel (0-15). Each tick = 1/4096 of the cycle."""
        on = max(0, min(4095, int(on)))
        off = max(0, min(4095, int(off)))
        data = ustruct.pack('<HH', on, off)
        self.i2c.writeto_mem(self.address, _LED0_ON_L + 4 * channel, data)

    def set_servo_pulse(self, channel, pulse_us, freq=50):
        """Set servo pulse width in microseconds."""
        period_us = 1000000.0 / freq  # 20000.0us for 50Hz
        ticks = int((pulse_us * 4096.0) / period_us + 0.5)
        self.set_pwm(channel, 0, ticks)
