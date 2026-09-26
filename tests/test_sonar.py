"""Ultrasonic module: same bytes as Hiwonder's HiwonderSDK/Sonar.py."""
import unittest

from robot_board.sonar import Sonar
from test_board import FakeBus


class SonarTests(unittest.TestCase):
    def test_distance(self):
        bus = FakeBus(reads=[(1093).to_bytes(2, "little")])
        self.assertEqual(Sonar(bus).distance_mm(), 1093)
        self.assertEqual(bus.writes, [[0]])

    def test_garbage_is_retried_never_reported_as_free_space(self):
        bus = FakeBus(reads=[(65439).to_bytes(2, "little")] * 2 + [(327).to_bytes(2, "little")])
        self.assertEqual(Sonar(bus).distance_mm(), 327)
        bus = FakeBus(reads=[(65439).to_bytes(2, "little")] * 10)
        self.assertIsNone(Sonar(bus).distance_mm())

    def test_distance_retries_then_none(self):
        bus = FakeBus(reads=[OSError(121, "Remote I/O error")] * 5)
        self.assertIsNone(Sonar(bus).distance_mm())

    def test_color(self):
        # Sonar.setPixelColor(1, Color(10, 20, 30)): regs 6, 7, 8 via write_byte_data
        bus = FakeBus()
        Sonar(bus).set_color(1, 10, 20, 30)
        self.assertEqual(bus.writes, [[2, 0], [6, 10], [7, 20], [8, 30]])

    def test_breathe(self):
        # Sonar.setBreathCycle(index 0, rgb 0..2, cycle ms) -> reg 9 + rgb, cycle // 100
        bus = FakeBus()
        Sonar(bus).breathe(0, (2000, 3300, 4700))
        self.assertEqual(bus.writes, [[2, 1], [9, 20], [10, 33], [11, 47]])


if __name__ == "__main__":
    unittest.main()
