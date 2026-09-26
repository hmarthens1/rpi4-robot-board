"""
Driver for the Hiwonder RaspberryPi-Adapter-4chMotorDrive V3.x expansion board
on a Raspberry Pi 4 running Ubuntu 22.04.

    from robot_board import Board
    with Board() as board:          # stops the motors when the block ends
        print(board.battery_v())
        board.set_motor(1, 30)

GPIO parts (RGB LEDs, buzzer, keys, LED1/LED2) are in robot_board.peripherals.
"""
from .board import Board, I2C_ADDR, I2C_BUS, MOTORS, SERVOS

__all__ = ["Board", "I2C_ADDR", "I2C_BUS", "MOTORS", "SERVOS"]
__version__ = "0.1.0"
