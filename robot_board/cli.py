"""
robot-board: check the expansion board from the command line.

    robot-board battery              # no sudo needed
    sudo robot-board test            # battery, RGB LEDs, buzzer, LED1/LED2 - nothing moves
    sudo robot-board test --motors   # also turns each motor: wheels off the table!
    robot-board keys                 # prints key presses until Ctrl+C
    robot-board sonar                # ultrasonic distance, 5 times a second, until Ctrl+C
    robot-board offsets              # the arm calibration saved on this robot (servo deviation)
    sudo robot-board offsets --set 5=-20 6=10   # change it by hand (restart robot-command after)
"""
import argparse
import os
import sys
import time

from .board import Board

OFFSETS_FILE = "/var/lib/robot_board/servo_offsets.json"

MOTOR_SPEED = 30     # percent
MOTOR_TIME = 1.0     # seconds each way


def cmd_battery(_args):
    v = Board().battery_v()
    if v is None:
        print("no valid reading from the board at I2C 0x7A - fitted and switched ON?")
        return 1
    print(f"{v:.2f} V")
    return 0


def cmd_test(args):
    from . import peripherals as io

    if os.geteuid() != 0:
        print("Run with sudo: the RGB LEDs need /dev/mem.")
        return 1
    failed = False
    board = Board()

    v = board.battery_v()
    if v is None:
        print("FAIL  battery: no valid reading from I2C 0x7A - board fitted and switched ON?")
        failed = True
    else:
        print(f"PASS  battery: {v:.2f} V")

    try:
        print("....  RGB LEDs: red, green, blue, then off")
        rgb = io.RGB()
        for color in ((80, 0, 0), (0, 80, 0), (0, 0, 80)):
            rgb.fill(*color)
            time.sleep(0.7)
        rgb.off()

        print("....  buzzer: two short beeps")
        io.Buzzer().beep(0.1, times=2, gap=0.2)

        print("....  LED1 then LED2, half a second each")
        leds = io.Leds()
        for n in (1, 2):
            leds.set(n, True)
            time.sleep(0.5)
            leds.set(n, False)

        if args.motors:
            print(f"....  motors 1-4 at {MOTOR_SPEED}%: forward, back, stop - one at a time")
            with board:   # stops all motors on the way out, also on Ctrl+C
                for m in (1, 2, 3, 4):
                    print(f"      motor {m}")
                    board.set_motor(m, MOTOR_SPEED)
                    time.sleep(MOTOR_TIME)
                    board.set_motor(m, -MOTOR_SPEED)
                    time.sleep(MOTOR_TIME)
                    board.set_motor(m, 0)
                    time.sleep(0.3)
    finally:
        io.cleanup()

    print("\nFix the FAIL item first." if failed else
          "\nDid the RGB LEDs, the buzzer and LED1/LED2 all work? Then the board is fine.")
    return 1 if failed else 0


def cmd_keys(_args):
    from . import peripherals as io

    keys = io.Keys()
    print("Press Key1 or Key2 on the board (Ctrl+C to stop)")
    held = {1: False, 2: False}
    try:
        while True:
            for k in (1, 2):
                now = keys.pressed(k)
                if now and not held[k]:
                    print(f"Key{k} pressed")
                held[k] = now
            time.sleep(0.02)
    except KeyboardInterrupt:
        pass
    finally:
        io.cleanup()
    return 0


def cmd_sonar(_args):
    from .sonar import Sonar

    sonar = Sonar()
    if sonar.distance_mm() is None:
        print("no answer from the ultrasonic module at I2C 0x77 - plugged into P7/P8/P9?")
        return 1
    sonar.fill(0, 0, 60)
    print("Distance in mm (Ctrl+C to stop)", flush=True)
    try:
        while True:
            mm = sonar.distance_mm()
            print(f"{mm:5d} mm" if mm is not None else "  --- no valid reading", flush=True)
            time.sleep(0.2)
    except KeyboardInterrupt:
        pass
    finally:
        sonar.off()
    return 0


def cmd_offsets(args):
    import json

    try:
        with open(OFFSETS_FILE) as f:
            offsets = json.load(f)
    except FileNotFoundError:
        offsets = {}
    if args.set:
        for item in args.set:
            servo, _, value = item.partition("=")
            if servo not in ("1", "2", "3", "4", "5", "6") or not value.lstrip("-").isdigit():
                print(f"bad '{item}': use SERVO=US, e.g. 5=-20")
                return 1
            if not -150 <= int(value) <= 150:
                print(f"bad '{item}': offsets are limited to -150..150 us")
                return 1
            offsets[servo] = int(value)
        os.makedirs(os.path.dirname(OFFSETS_FILE), exist_ok=True)
        with open(OFFSETS_FILE, "w") as f:
            f.write(json.dumps(offsets) + "\n")
        print("saved - run 'sudo systemctl restart robot-command' to use it")
    if not offsets:
        print(f"no calibration saved ({OFFSETS_FILE}): every offset is 0")
    for servo in sorted(offsets, key=int):
        print(f"servo {servo}: {offsets[servo]:+d} us")
    return 0


def main(argv=None):
    parser = argparse.ArgumentParser(prog="robot-board", description=__doc__.split("\n\n")[0])
    sub = parser.add_subparsers(dest="cmd", required=True)
    sub.add_parser("battery", help="print the battery voltage")
    t = sub.add_parser("test", help="battery, RGB, buzzer, LEDs (needs sudo)")
    t.add_argument("--motors", action="store_true", help="also turn each motor (lift the robot first)")
    sub.add_parser("keys", help="print Key1/Key2 presses")
    sub.add_parser("sonar", help="print the ultrasonic distance")
    o = sub.add_parser("offsets", help="print (or --set) the saved arm servo offsets")
    o.add_argument("--set", nargs="+", metavar="SERVO=US", help="e.g. 5=-20 6=10 (needs sudo)")
    args = parser.parse_args(argv)
    commands = {"battery": cmd_battery, "test": cmd_test, "keys": cmd_keys, "sonar": cmd_sonar,
                "offsets": cmd_offsets}
    return commands[args.cmd](args)


if __name__ == "__main__":
    sys.exit(main())
