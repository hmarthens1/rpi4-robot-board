"""
Mecanum-wheel chassis on top of Board (only for robots with mecanum wheels).

    motor1 |  ^  | motor2        vx   > 0: slide right
           |     |               vy   > 0: forward
    motor3 |     | motor4        turn > 0: rotate counter-clockwise (left)

All three are in the board's units, percent of full motor speed. Converting to
m/s and rad/s needs each robot's measured wheel speed, so it isn't done here.
Same wheel mixing as Hiwonder's mecanum.py (rollers in an X seen from above).
"""


def wheel_speeds(vx, vy, turn):
    """Motor 1-4 speeds in percent, scaled down together if any exceeds 100."""
    speeds = [vy + vx - turn, vy - vx + turn, vy - vx - turn, vy + vx + turn]
    biggest = max(abs(s) for s in speeds)
    if biggest > 100:
        speeds = [s * 100 / biggest for s in speeds]   # keep the direction
    return [round(s) for s in speeds]


class Mecanum:
    def __init__(self, board):
        self.board = board

    def drive(self, vx=0, vy=0, turn=0):
        self.board.set_motors(*wheel_speeds(vx, vy, turn))

    def stop(self):
        self.board.stop()
