"""
The expansion board's microcontroller: 4 DC motors, 6 PWM servos, battery voltage.

The Pi talks to it over I2C bus 1 at address 0x7A. That address is above 0x77,
so `i2cdetect` never shows it; `robot_check.sh` reads the battery register
instead.

Register map (the same protocol as Hiwonder's HiwonderSDK/Board.py):

    reg 0       battery voltage: write [0], then read 2 bytes (mV, little-endian)
                as a SEPARATE transfer - a combined write+read returns garbage
    reg 31..34  motor 1..4 speed: write [reg, int8 -100..100]
    reg 40      servo move: write [40, count, time_lo, time_hi,
                                   id, pulse_lo, pulse_hi, (id, pulse_lo, pulse_hi)...]
                pulse 500..2500 us, time 0..30000 ms

Nothing here needs root: /dev/i2c-1 belongs to a group the default user is in.
"""
import time

try:
    from smbus2 import SMBus, i2c_msg
except ImportError:  # lets the tests and the docs import this module anywhere
    SMBus = i2c_msg = None

I2C_BUS = 1
I2C_ADDR = 0x7A

REG_BATTERY = 0
REG_MOTOR = 31          # + motor index 0..3
REG_SERVO_MOVE = 40

MOTORS = (1, 2, 3, 4)
SERVOS = (1, 2, 3, 4, 5, 6)
PULSE_MIN, PULSE_MAX = 500, 2500
MOVE_TIME_MAX = 30000

# Hiwonder's SDK flips motors 1 and 3, so a positive speed turns all four
# wheels "forward" on their chassis. Override per robot if yours is wired
# differently (see board_test.py --motors).
DEFAULT_MOTOR_POLARITY = {1: -1, 2: 1, 3: -1, 4: 1}


class I2CTransport:
    """Raw I2C writes and reads, one transfer each, with a retry like the SDK's."""

    def __init__(self, bus=I2C_BUS, addr=I2C_ADDR, retries=2):
        if SMBus is None:
            raise ImportError("smbus2 is not installed: sudo pip3 install smbus2")
        self.bus, self.addr, self.retries = bus, addr, retries

    def _rdwr(self, msg):
        for attempt in range(self.retries + 1):
            try:
                with SMBus(self.bus) as bus:
                    bus.i2c_rdwr(msg)
                return
            except OSError:
                if attempt == self.retries:
                    raise
                time.sleep(0.002)

    def write(self, data):
        self._rdwr(i2c_msg.write(self.addr, list(data)))

    def read(self, n):
        msg = i2c_msg.read(self.addr, n)
        self._rdwr(msg)
        return bytes(list(msg))


def _clamp(value, low, high):
    return max(low, min(high, value))


class Board:
    """
    >>> board = Board()
    >>> board.battery_v()
    12.22
    >>> board.set_motor(1, 40)      # percent, -100..100
    >>> board.stop()
    >>> board.set_servo_pulse(1, 1500, ms=500)
    """

    def __init__(self, transport=None, motor_polarity=None, servo_offsets=None):
        """
        transport:      anything with write(bytes) and read(n); default: real I2C
        motor_polarity: {motor: +1 or -1}; default DEFAULT_MOTOR_POLARITY
        servo_offsets:  {servo: microseconds added to every pulse}, for trimming
                        a servo's centre; servos not listed get 0
        """
        self.io = transport or I2CTransport()
        self.motor_polarity = dict(DEFAULT_MOTOR_POLARITY)
        self.motor_polarity.update(motor_polarity or {})
        self.servo_offsets = dict(servo_offsets or {})
        self.motor_speeds = {m: 0 for m in MOTORS}
        self.servo_pulses = {s: None for s in SERVOS}

    # ------------------------------------------------------------- battery
    def battery_mv(self, tries=6, low=3000, high=20000):
        """Battery voltage in mV, or None. Retries: some reads come back garbled."""
        for _ in range(tries):
            try:
                self.io.write([REG_BATTERY])
                raw = self.io.read(2)
            except OSError:
                continue
            mv = int.from_bytes(raw, "little")
            if low <= mv <= high:
                return mv
        return None

    def battery_v(self):
        mv = self.battery_mv()
        return None if mv is None else round(mv / 1000, 2)

    # -------------------------------------------------------------- motors
    def set_motor(self, motor, speed):
        """Speed in percent, -100..100. Positive = forward (after polarity)."""
        if motor not in MOTORS:
            raise ValueError(f"motor must be 1-4, not {motor}")
        speed = int(_clamp(speed, -100, 100))
        raw = speed * self.motor_polarity[motor]
        self.io.write([REG_MOTOR + motor - 1, raw & 0xFF])
        self.motor_speeds[motor] = speed

    def set_motors(self, s1, s2, s3, s4):
        for motor, speed in zip(MOTORS, (s1, s2, s3, s4)):
            self.set_motor(motor, speed)

    def stop(self):
        """Stop all four motors. Keeps going if one write fails."""
        error = None
        for motor in MOTORS:
            try:
                self.set_motor(motor, 0)
            except OSError as e:
                error = e
        if error:
            raise error

    # -------------------------------------------------------------- servos
    def set_servo_pulses(self, pulses, ms=1000):
        """pulses: {servo: microseconds 500..2500}. All move together over `ms`."""
        if not pulses:
            return
        ms = int(_clamp(ms, 0, MOVE_TIME_MAX))
        buf = [REG_SERVO_MOVE, len(pulses)] + list(ms.to_bytes(2, "little"))
        for servo, pulse in pulses.items():
            if servo not in SERVOS:
                raise ValueError(f"servo must be 1-6, not {servo}")
            pulse = int(_clamp(pulse + self.servo_offsets.get(servo, 0), PULSE_MIN, PULSE_MAX))
            buf += [servo] + list(pulse.to_bytes(2, "little"))
            self.servo_pulses[servo] = pulse
        self.io.write(buf)

    def set_servo_pulse(self, servo, pulse, ms=1000):
        self.set_servo_pulses({servo: pulse}, ms)

    def set_servo_angle(self, servo, angle, ms=1000):
        """Angle 0..180 degrees, mapped linearly onto 500..2500 us."""
        angle = _clamp(angle, 0, 180)
        self.set_servo_pulse(servo, PULSE_MIN + angle * (PULSE_MAX - PULSE_MIN) / 180, ms)

    # ------------------------------------------------------------- context
    def __enter__(self):
        return self

    def __exit__(self, *exc):
        self.stop()   # never leave the motors running when a script ends
        return False
