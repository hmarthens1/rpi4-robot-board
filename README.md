# rpi4-robot-board

Python driver for the **Hiwonder RaspberryPi-Adapter-4chMotorDrive V3.x** expansion board
on the fleet's **Raspberry Pi 4** robots (`robot01`–`robot03`), running **Ubuntu Server 22.04**.
Setup of the Pis themselves: [rpi4-ros2-multirobot](https://github.com/hmarthens1/rpi4-ros2-multirobot).

It speaks the same I2C protocol as Hiwonder's `HiwonderSDK/Board.py` (the tests check it sends
identical bytes), without that SDK's hardcoded paths, import-time side effects and broken functions.

## Install (on each robot)

```bash
git clone https://github.com/hmarthens1/rpi4-robot-board.git ~/rpi4-robot-board
cd ~/rpi4-robot-board
sudo bash scripts/install.sh
```

Update later with `git pull && sudo bash scripts/install.sh`.

## Check the board

```bash
robot-board battery                # e.g. 8.17 V (2x 18650) - no sudo needed
sudo robot-board test              # battery, RGB LEDs, buzzer, LED1/LED2 - nothing moves
sudo robot-board test --motors     # each motor forward/back at 30 %: lift the robot first!
robot-board keys                   # prints Key1/Key2 presses, Ctrl+C to stop
robot-board sonar                  # ultrasonic distance in mm, Ctrl+C to stop
robot-board offsets                # arm calibration (servo deviation) saved on this robot
```

## Use it

```python
from robot_board import Board

with Board() as board:             # stops all motors when the block ends, also on errors
    print(board.battery_v())       # 8.17
    board.set_motor(1, 40)         # percent, -100..100
    board.set_motors(30, 30, 30, 30)
    board.set_servo_pulse(1, 1500, ms=500)       # 500..2500 us, move over 500 ms
    board.set_servo_angle(3, 45)                 # 0..180 deg
    board.set_servo_pulses({1: 1000, 3: 2000}, ms=800)   # several at once
```

```python
from robot_board.peripherals import RGB, Buzzer, Keys, Leds, cleanup

RGB().fill(0, 0, 80)               # needs sudo (rpi_ws281x uses /dev/mem)
Buzzer().beep(0.1, times=2)
Keys().pressed(1)                  # True while Key1 is held
Leds().set(1, True)                # LED1 on
cleanup()
```

Ultrasonic module (Hiwonder I2C sonar at `0x77`, on port P7/P8/P9):

```python
from robot_board.sonar import Sonar
sonar = Sonar()
sonar.distance_mm()                # 1093 (capped at 5000), or None
sonar.fill(0, 0, 60)               # its two RGB LEDs; sonar.breathe(0) for breathing
```

Mecanum wheels only:

```python
from robot_board.mecanum import Mecanum
Mecanum(board).drive(vx=0, vy=40, turn=0)      # forward at 40 %
```

Per-robot trims: `Board(motor_polarity={3: 1}, servo_offsets={5: -64, 6: -47})`.

## The board

| Function | Interface | In this package |
|---|---|---|
| 4 DC motors | I2C `0x7A`, registers 31–34, int8 −100…100 | `Board.set_motor`, `set_motors`, `stop` |
| 6 PWM servos | I2C `0x7A`, register 40: `[40, n, t_lo, t_hi, id, p_lo, p_hi, …]` | `Board.set_servo_pulse(s)`, `set_servo_angle` |
| Battery | I2C `0x7A`, register 0: write `[0]`, then a **separate** 2-byte read (mV) | `Board.battery_mv`, `battery_v` |
| 2× RGB (WS2812) | GPIO12, `rpi_ws281x`, needs root | `peripherals.RGB` |
| Buzzer | GPIO6 | `peripherals.Buzzer` |
| Key1, Key2 | GPIO13, GPIO23, to GND | `peripherals.Keys` |
| LED1, LED2 | GPIO16, GPIO26, active low | `peripherals.Leds` |
| Ultrasonic module (add-on) | I2C `0x77`: reg 0 distance (mm), regs 2–14 its LEDs | `sonar.Sonar` |
| Serial bus servos, port P12 | header UART GPIO14/15 | not yet (the UART is Ubuntu's serial console) |

**`i2cdetect` never shows the board**: `0x7A` is above `0x77`, the top of its scan. Read it with
`i2ctransfer -a -y 1 w1@0x7a 0x00 && i2ctransfer -a -y 1 r2@0x7a`. About one battery read in
three comes back garbled; `battery_mv` retries until the value is plausible.

## Differences from HiwonderSDK/Board.py

| HiwonderSDK | here |
|---|---|
| Must live in `~/mse112-ws-student/MasterPi` | pip package, import from anywhere |
| Importing it starts the RGB driver, so everything needs `sudo` | only the RGB LEDs need `sudo` |
| `setPWMServoAngle` fails with `NameError` | `set_servo_angle` works (through the pulse command) |
| Bus-servo functions fail: their module isn't included | left out until bus servos are wired up |
| `setPWMServoPulse(2, …)` raises `KeyError` (no offset for servo 2 in `Deviation.yaml`) | missing offsets default to 0 |
| Motors keep running if a script crashes | `with Board()` stops them |

## The robot's ROS 2 nodes (C++)

`ros2/robot_status` is an `ament_cmake` package with the board driver in C++ (`include/robot_board`,
`src/`) and two nodes, run as systemd services by `scripts/install_status_service.sh`:

| Service | Node | Topics (in `/<hostname>`) |
|---|---|---|
| `robot-status` | `status_node` | `battery`, `system`, `sonar/range` |
| `robot-command` | `command_node` (root, for the RGB LEDs) | `command` → `command_result`, `cmd_vel` |

`command_node` takes JSON commands (`drive`, `motor`, `servo`, `led`, `rgb`, `buzzer`, `stop`,
`status`) and enforces its own limits: speed cap, every motion stops by itself after at most
`max_duration`, `stop` always wins, and the limits can't be changed with `ros2 param set`.
With the ultrasonic module plugged in, a forward move is refused when an obstacle is closer than
`min_clearance` (0.3 m) or the distance is unknown, and stopped if it gets that close. Both nodes
look for the module every 5 s, so it can be plugged in or out while the robot runs.

The C++ driver speaks the same bytes as the Python one (same test cases) and shares its I2C lock
file, so the nodes and the `robot-board` command-line tool can run at the same time.
On a Pi 4 each node uses about 0.6 % of a core and 20 MB (the earlier Python nodes: 3–5 % and 55 MB).
The RGB LED library, [rpi_ws281x](https://github.com/jgarff/rpi_ws281x), is fetched at build time
at a pinned commit (it isn't packaged for Ubuntu).

### The arm

On a robot with the 5-servo arm (1 gripper, 3 wrist, 4 elbow, 5 shoulder, 6 base; 2 is the fan),
`command_node` also takes `arm_pose` (`stand` = every servo at 1500 us, `fold`, `rest`, `ready`),
`arm_move` (gripper tip to x, y, z in cm with inverse kinematics ported from MasterPi's ArmIK and
checked against it on 3564 cases), `gripper`, `arm_servos` (raw pulses), `arm_sequence`
(frames, e.g. a MasterPi `.d6a` action group), and `arm_release` (stop driving servos: they go limp).

**Calibration (servo deviation).** Each robot keeps its own offsets, in us added to every pulse
(-150..150), in `/var/lib/robot_board/servo_offsets.json`; there is no default deviation.
To read or change them:

| Where | Read | Change |
|---|---|---|
| Dashboard, Arm tab | **Read from robot** (also done when you pick a robot) | spin boxes, **Apply** / **Save on robot** |
| ROS 2 | `{"action": "get_servo_offsets"}` → `{"offsets", "saved", "file"}`; also in `status` | `{"action": "set_servo_offsets", "offsets": {"5": -20}, "save": true}` |
| Shell on the robot | `robot-board offsets` | `sudo robot-board offsets --set 5=-20 6=10`, then `sudo systemctl restart robot-command` |

```bash
ros2 topic pub --once /robot01/command std_msgs/String '{data: "{\"action\": \"get_servo_offsets\"}"}'
ros2 topic echo --once /robot01/command_result
```

To calibrate: **Stand** (all 1500), then nudge each joint's offset until the arm is straight up
and the gripper centred, and save.

## Tests

```bash
python3 -m unittest discover -s tests -v     # Python driver, anywhere, no Pi needed
colcon test --packages-select robot_status   # C++ driver (gtest), in ~/ros2_ws on a robot
```
