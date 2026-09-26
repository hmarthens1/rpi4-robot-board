"""
Hiwonder I2C ultrasonic module (the "glowy" one with two RGB LEDs), plugged
into one of the board's I2C ports (P7/P8/P9). It shows up in i2cdetect at 0x77.

Register map (same protocol as Hiwonder's HiwonderSDK/Sonar.py):

    reg 0       distance: write [0], then read 2 bytes as a separate transfer (mm, little-endian)
    reg 2       LED mode: 0 = fixed colour, 1 = breathing
    reg 3..5    LED 1 red, green, blue (0..255)
    reg 6..8    LED 2 red, green, blue
    reg 9..11   LED 1 breathing period per colour, in units of 100 ms
    reg 12..14  LED 2 breathing period per colour

No root needed.
"""
from .board import I2CTransport

SONAR_ADDR = 0x77
REG_DISTANCE = 0
REG_MODE = 2
REG_COLOR = (3, 6)          # first register of LED 1 and LED 2
REG_BREATH = (9, 12)
MAX_MM = 5000               # the module reports up to about 5 m


class Sonar:
    """
    >>> sonar = Sonar()
    >>> sonar.distance_mm()
    1093
    >>> sonar.set_color(0, 0, 80, 0)      # LED 1 green
    """

    def __init__(self, transport=None):
        self.io = transport or I2CTransport(addr=SONAR_ADDR)

    def distance_mm(self, tries=3):
        """Distance in mm (capped at 5000), or None if the module doesn't answer."""
        for _ in range(tries):
            try:
                self.io.write([REG_DISTANCE])
                return min(int.from_bytes(self.io.read(2), "little"), MAX_MM)
            except OSError:
                continue
        return None

    def distance_m(self):
        mm = self.distance_mm()
        return None if mm is None else mm / 1000

    def _write_reg(self, reg, value):
        self.io.write([reg, int(value) & 0xFF])

    def set_color(self, led, r, g, b):
        """led 0 or 1. Also switches the LEDs to fixed-colour mode."""
        if led not in (0, 1):
            raise ValueError("led must be 0 or 1")
        self._write_reg(REG_MODE, 0)
        for offset, value in enumerate((r, g, b)):
            self._write_reg(REG_COLOR[led] + offset, value)

    def fill(self, r, g, b):
        for led in (0, 1):
            self.set_color(led, r, g, b)

    def off(self):
        self.fill(0, 0, 0)

    def breathe(self, led, period_ms=(2000, 3300, 4700)):
        """Breathing mode: led 0 or 1, one period per colour (red, green, blue), in ms."""
        if led not in (0, 1):
            raise ValueError("led must be 0 or 1")
        self._write_reg(REG_MODE, 1)
        for offset, ms in enumerate(period_ms):
            self._write_reg(REG_BREATH[led] + offset, ms // 100)
