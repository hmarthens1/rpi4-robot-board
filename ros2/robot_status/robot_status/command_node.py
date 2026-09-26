"""
robot_command: the robot's command listener. It turns commands from the
laptop (the voice agent, or anything else) into motor, servo, LED and buzzer
actions, with safety limits that hold whatever the sender asks for.

    ros2 run robot_status command_node --ros-args -r __ns:=/robot01

Topics (relative to the namespace):

    command         std_msgs/String   in:  one JSON command (see below)
    command_result  std_msgs/String   out: JSON {"id", "ok", "action", "detail"}
    cmd_vel         geometry_msgs/Twist in: continuous driving; stops 0.5 s after
                                        the last message (joystick / teleop)

Commands ("id" is optional and is echoed back in the result):

    {"action": "drive", "vx": 0, "vy": 0.4, "turn": 0, "duration": 1.5}
        vx = slide right, vy = forward, turn = counter-clockwise; each -1..1 of
        the speed cap. Strafing (vx) needs the parameter drive:=mecanum.
    {"action": "motor", "motor": 1, "speed": 0.3, "duration": 1.0}   speed -1..1
    {"action": "servo", "servo": 1, "pulse": 1500, "ms": 500}          or "angle": 0..180
    {"action": "led", "led": 1, "on": true}                            LED1 / LED2
    {"action": "rgb", "r": 0, "g": 0, "b": 80}                         needs the node to run as root
    {"action": "buzzer", "seconds": 0.2, "times": 2}
    {"action": "stop"}                                                 always accepted
    {"action": "status"}                                               battery, distance, what's running

Safety: speeds are scaled to max_speed (percent), motions last at most
max_duration seconds and then stop by themselves, a new motion replaces the
running one, and the motors stop when the node exits.
"""
import json
import threading
import time

import rclpy
from rclpy.executors import ExternalShutdownException
from geometry_msgs.msg import Twist
from rclpy.node import Node
from std_msgs.msg import String

from robot_board import Board
from robot_board.mecanum import wheel_speeds
from robot_board.sonar import Sonar


def _clip(x, low=-1.0, high=1.0):
    return max(low, min(high, float(x)))


class CommandNode(Node):
    def __init__(self):
        super().__init__("robot_command")
        self.declare_parameter("max_speed", 50.0)      # percent of full motor speed
        self.declare_parameter("max_duration", 5.0)    # seconds per motion command
        self.declare_parameter("drive", "mecanum")     # "mecanum" or "differential"
        self.declare_parameter("cmd_vel_timeout", 0.5)

        self.board = Board()
        self.lock = threading.Lock()
        self.motion_until = 0.0         # monotonic time the current motion ends
        self.motion_desc = "stopped"
        self.cmd_vel_last = 0.0
        self._peripherals()
        try:
            self.sonar = Sonar() if Sonar().distance_mm() is not None else None
        except OSError:
            self.sonar = None

        self.result_pub = self.create_publisher(String, "command_result", 10)
        self.create_subscription(String, "command", self.on_command, 10)
        self.create_subscription(Twist, "cmd_vel", self.on_cmd_vel, 10)
        self.create_timer(0.05, self.watchdog)
        self.board.stop()
        self.get_logger().info(
            f"ready: max_speed {self.p('max_speed')} %, max_duration {self.p('max_duration')} s, "
            f"drive {self.p('drive')}, RGB {'yes' if self.rgb else 'no (not root)'}, "
            f"sonar {'yes' if self.sonar else 'no'}")

    def p(self, name):
        return self.get_parameter(name).value

    def _peripherals(self):
        from robot_board import peripherals
        self.peripherals = peripherals
        self.leds = peripherals.Leds()
        self.buzzer = peripherals.Buzzer()
        try:
            self.rgb = peripherals.RGB()
        except Exception:        # rpi_ws281x needs /dev/mem, i.e. root
            self.rgb = None

    # ------------------------------------------------------------ plumbing
    def reply(self, cmd_id, action, ok, detail):
        msg = String()
        msg.data = json.dumps({"id": cmd_id, "action": action, "ok": ok, "detail": detail})
        self.result_pub.publish(msg)
        # Two separate call sites: rclpy refuses one call site that switches
        # between severities ("Logger severity cannot be changed between calls").
        if ok:
            self.get_logger().info(f"{action}: {detail}")
        else:
            self.get_logger().warn(f"{action}: {detail}")

    def on_command(self, msg):
        cmd_id, action = None, "?"
        try:
            cmd = json.loads(msg.data)
            cmd_id, action = cmd.get("id"), str(cmd.get("action", "?"))
            handler = getattr(self, f"do_{action}", None)
            if handler is None:
                raise ValueError(f"unknown action '{action}'")
            self.reply(cmd_id, action, True, handler(cmd))
        except Exception as e:   # bad JSON, bad values, I2C errors: report, never crash
            if action not in ("stop",):
                self._stop_motors()
            self.reply(cmd_id, action, False, f"{type(e).__name__}: {e}")

    def _duration(self, cmd):
        d = float(cmd.get("duration", 1.0))
        if d <= 0:
            raise ValueError("duration must be > 0")
        return min(d, self.p("max_duration"))

    def _run_motion(self, speeds, duration, desc):
        with self.lock:
            self.board.set_motors(*speeds)
            self.motion_until = time.monotonic() + duration
            self.motion_desc = desc
        return f"{desc} for {duration:.1f} s (motors {speeds})"

    def _stop_motors(self):
        with self.lock:
            self.motion_until = 0.0
            self.motion_desc = "stopped"
            self.board.stop()

    def watchdog(self):
        now = time.monotonic()
        with self.lock:
            running = self.motion_until > 0
            expired = running and now >= self.motion_until
            vel_stale = self.cmd_vel_last and now - self.cmd_vel_last > self.p("cmd_vel_timeout")
        if expired:
            self._stop_motors()
            self.reply(None, "stop", True, "motion finished")
        elif vel_stale:
            self.cmd_vel_last = 0.0
            self._stop_motors()

    # ------------------------------------------------------------ actions
    def do_drive(self, cmd):
        vx, vy, turn = (_clip(cmd.get(k, 0)) for k in ("vx", "vy", "turn"))
        if vx and self.p("drive") != "mecanum":
            raise ValueError("this robot can't slide sideways (drive is not mecanum)")
        scale = self.p("max_speed")
        speeds = wheel_speeds(vx * scale, vy * scale, turn * scale)
        return self._run_motion(speeds, self._duration(cmd), f"drive vx={vx:+.2f} vy={vy:+.2f} turn={turn:+.2f}")

    def do_motor(self, cmd):
        motor = int(cmd["motor"])
        if motor not in (1, 2, 3, 4):
            raise ValueError("motor must be 1-4")
        speed = round(_clip(cmd.get("speed", 0.3)) * self.p("max_speed"))
        speeds = [0, 0, 0, 0]
        speeds[motor - 1] = speed
        return self._run_motion(speeds, self._duration(cmd), f"motor {motor} at {speed} %")

    def do_servo(self, cmd):
        servo, ms = int(cmd["servo"]), int(cmd.get("ms", 500))
        if "angle" in cmd:
            angle = _clip(cmd["angle"], 0, 180)
            self.board.set_servo_angle(servo, angle, ms)
            return f"servo {servo} to {angle:.0f} deg over {ms} ms"
        pulse = int(_clip(cmd.get("pulse", 1500), 500, 2500))
        self.board.set_servo_pulse(servo, pulse, ms)
        return f"servo {servo} to {pulse} us over {ms} ms"

    def do_led(self, cmd):
        led, on = int(cmd.get("led", 1)), bool(cmd.get("on", True))
        if led not in (1, 2):
            raise ValueError("led must be 1 or 2")
        self.leds.set(led, on)
        return f"LED{led} {'on' if on else 'off'}"

    def do_rgb(self, cmd):
        if self.rgb is None:
            raise RuntimeError("RGB LEDs need the node to run as root")
        r, g, b = (int(_clip(cmd.get(k, 0), 0, 255)) for k in "rgb")
        self.rgb.fill(r, g, b)
        return f"RGB set to ({r}, {g}, {b})"

    def do_buzzer(self, cmd):
        seconds = _clip(cmd.get("seconds", 0.2), 0.02, 2.0)
        times = int(_clip(cmd.get("times", 1), 1, 5))
        self.buzzer.beep(seconds, times=times, gap=0.15)
        return f"beeped {times}x {seconds:.2f} s"

    def do_stop(self, _cmd):
        self._stop_motors()
        return "all motors stopped"

    def do_status(self, _cmd):
        mm = self.sonar.distance_mm() if self.sonar else None
        return {"battery_v": self.board.battery_v(),
                "distance_m": None if mm is None else mm / 1000,
                "motion": self.motion_desc,
                "motor_speeds": self.board.motor_speeds,
                "servo_pulses": self.board.servo_pulses,
                "max_speed": self.p("max_speed"), "max_duration": self.p("max_duration"),
                "drive": self.p("drive")}

    # ---------------------------------------------------------- cmd_vel
    def on_cmd_vel(self, msg):
        vx = _clip(-msg.linear.y) if self.p("drive") == "mecanum" else 0.0   # ROS: +y is left
        vy, turn = _clip(msg.linear.x), _clip(msg.angular.z)
        scale = self.p("max_speed")
        with self.lock:
            self.board.set_motors(*wheel_speeds(vx * scale, vy * scale, turn * scale))
            self.motion_until = 0.0
            self.motion_desc = "cmd_vel"
            self.cmd_vel_last = time.monotonic()

    def shutdown(self):
        try:
            self.board.stop()
            if self.rgb:
                self.rgb.off()
            self.peripherals.cleanup()
        except Exception:
            pass


def main():
    rclpy.init()
    node = CommandNode()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):   # Ctrl+C or systemctl stop
        pass
    finally:
        node.shutdown()
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == "__main__":
    main()
