"""
Protocol tests: robot_board must send exactly the bytes that Hiwonder's
HiwonderSDK/Board.py sends. The expected values below are written the way
Board.py builds them. Runs anywhere (no Pi, no I2C):

    python3 -m pytest tests      # or: python3 -m unittest discover tests
"""
import unittest

from robot_board.board import Board
from robot_board.mecanum import wheel_speeds


class FakeBus:
    """Records writes; answers reads from a queue (default: 12.22 V)."""

    def __init__(self, reads=None):
        self.writes = []
        self.reads = list(reads or [])

    def write(self, data):
        self.writes.append(list(data))

    def read(self, n):
        item = self.reads.pop(0) if self.reads else (12220).to_bytes(2, "little")
        if isinstance(item, Exception):
            raise item
        return item


def hiwonder_motor(index, speed):
    """Board.setMotor: motors 1 and 3 negated, reg 31+i, int8."""
    if index not in (2, 4):
        speed = -speed
    speed = max(-100, min(100, speed))
    return [31 + index - 1, speed.to_bytes(1, "little", signed=True)[0]]


def hiwonder_servo(servo_id, pulse, use_time, deviation=0):
    """Board.setPWMServoPulse."""
    pulse = max(500, min(2500, pulse + deviation))
    use_time = max(0, min(30000, use_time))
    return [40, 1] + list(use_time.to_bytes(2, "little")) + [servo_id] + list(pulse.to_bytes(2, "little"))


class MotorTests(unittest.TestCase):
    def test_same_bytes_as_hiwonder(self):
        for motor in (1, 2, 3, 4):
            for speed in (-120, -100, -37, -1, 0, 1, 50, 100, 150):
                bus = FakeBus()
                Board(bus).set_motor(motor, speed)
                self.assertEqual(bus.writes, [hiwonder_motor(motor, speed)], (motor, speed))

    def test_polarity_override(self):
        bus = FakeBus()
        Board(bus, motor_polarity={1: 1}).set_motor(1, 40)
        self.assertEqual(bus.writes, [[31, 40]])

    def test_stop_writes_all_four(self):
        bus = FakeBus()
        Board(bus).stop()
        self.assertEqual(bus.writes, [[31, 0], [32, 0], [33, 0], [34, 0]])

    def test_context_manager_stops(self):
        bus = FakeBus()
        with Board(bus) as b:
            b.set_motor(2, 60)
        self.assertEqual(bus.writes[-4:], [[31, 0], [32, 0], [33, 0], [34, 0]])

    def test_bad_motor(self):
        with self.assertRaises(ValueError):
            Board(FakeBus()).set_motor(5, 10)


class ServoTests(unittest.TestCase):
    def test_same_bytes_as_hiwonder(self):
        for servo in (1, 6):
            for pulse, ms in ((1500, 1000), (400, 20), (2600, 40000), (500, 0)):
                bus = FakeBus()
                Board(bus).set_servo_pulse(servo, pulse, ms)
                self.assertEqual(bus.writes, [hiwonder_servo(servo, pulse, ms)])

    def test_offset(self):
        bus = FakeBus()
        Board(bus, servo_offsets={5: -64}).set_servo_pulse(5, 1500, 500)
        self.assertEqual(bus.writes, [hiwonder_servo(5, 1500, 500, deviation=-64)])

    def test_servo_2_without_offset(self):
        # Hiwonder's Deviation.yaml has no '2', so Board.py raises KeyError here.
        bus = FakeBus()
        Board(bus).set_servo_pulse(2, 1500)
        self.assertEqual(bus.writes, [hiwonder_servo(2, 1500, 1000)])

    def test_several_at_once(self):
        # Board.setPWMServosPulse([time, count, id1, p1, id2, p2])
        bus = FakeBus()
        Board(bus).set_servo_pulses({1: 1000, 3: 2000}, ms=800)
        self.assertEqual(bus.writes, [[40, 2, 0x20, 0x03, 1, 0xE8, 0x03, 3, 0xD0, 0x07]])

    def test_angle(self):
        bus = FakeBus()
        Board(bus).set_servo_angle(1, 90, ms=0)
        self.assertEqual(bus.writes, [hiwonder_servo(1, 1500, 0)])


class BatteryTests(unittest.TestCase):
    def test_write_then_separate_read(self):
        bus = FakeBus()
        self.assertEqual(Board(bus).battery_mv(), 12220)
        self.assertEqual(bus.writes, [[0]])

    def test_retries_garbled_and_failed_reads(self):
        bus = FakeBus(reads=[(65470).to_bytes(2, "little"), OSError(121, "Remote I/O error"),
                             (231).to_bytes(2, "little"), (12250).to_bytes(2, "little")])
        self.assertEqual(Board(bus).battery_mv(), 12250)

    def test_gives_up(self):
        bus = FakeBus(reads=[OSError(121, "Remote I/O error")] * 10)
        self.assertIsNone(Board(bus).battery_v())


class MecanumTests(unittest.TestCase):
    def test_matches_hiwonder_mixing(self):
        # mecanum.py: v1=vy+vx-vp, v2=vy-vx+vp, v3=vy-vx-vp, v4=vy+vx+vp, vp=-angular*(a+b)
        # Here turn plays the role of -vp (counter-clockwise positive).
        self.assertEqual(wheel_speeds(0, 50, 0), [50, 50, 50, 50])
        self.assertEqual(wheel_speeds(40, 0, 0), [40, -40, -40, 40])
        self.assertEqual(wheel_speeds(0, 0, 30), [-30, 30, -30, 30])

    def test_scales_down_together(self):
        self.assertEqual(wheel_speeds(100, 100, 0), [100, 0, 0, 100])
        self.assertEqual(wheel_speeds(0, 100, 100), [0, 100, 0, 100])


if __name__ == "__main__":
    unittest.main()
