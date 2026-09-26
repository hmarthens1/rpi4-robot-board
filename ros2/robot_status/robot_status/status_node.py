"""
robot_status: publish what a robot knows about itself. Read-only - it never
moves anything. Start it inside the robot's namespace:

    ros2 run robot_status status_node --ros-args -r __ns:=/robot01

Topics (relative to the namespace):

    battery       sensor_msgs/BatteryState      1 Hz  board battery voltage
    sonar/range   sensor_msgs/Range             5 Hz  only if the ultrasonic module answers
    system        diagnostic_msgs/DiagnosticStatus  1 Hz  CPU temperature and load,
                  memory, disk, Wi-Fi signal, IP, power flags, uptime

Parameters: battery_rate, sonar_rate, system_rate (Hz), battery_low_v (V),
temp_warn_c (deg C).
"""
import math
import os
import shutil
import socket
import subprocess

import rclpy
from rclpy.executors import ExternalShutdownException
from diagnostic_msgs.msg import DiagnosticStatus, KeyValue
from rclpy.node import Node
from sensor_msgs.msg import BatteryState, Range

from robot_board import Board
from robot_board.sonar import Sonar

THROTTLED = "/sys/devices/platform/soc/soc:firmware/get_throttled"


def _read(path, default=""):
    try:
        with open(path) as f:
            return f.read().strip()
    except OSError:
        return default


def cpu_temp_c():
    raw = _read("/sys/class/thermal/thermal_zone0/temp")
    return int(raw) / 1000 if raw else float("nan")


def mem_used_percent():
    info = {}
    for line in _read("/proc/meminfo").splitlines():
        key, value = line.split(":", 1)
        info[key] = int(value.split()[0])
    return 100 * (1 - info["MemAvailable"] / info["MemTotal"])


def wifi_signal_dbm():
    """From /proc/net/wireless: 'wlan0: 0000   70.  -40.  -256 ...'."""
    for line in _read("/proc/net/wireless").splitlines():
        if line.strip().startswith("wlan0:"):
            return float(line.split()[3].rstrip("."))
    return float("nan")


def wlan0_ip():
    try:
        out = subprocess.run(["ip", "-4", "-br", "addr", "show", "wlan0"],
                             capture_output=True, text=True, timeout=2).stdout.split()
        return out[2].split("/")[0] if len(out) > 2 else ""
    except (OSError, subprocess.SubprocessError):
        return ""


def throttled_flags():
    raw = _read(THROTTLED)
    return int(raw, 16) if raw else None


class StatusNode(Node):
    def __init__(self):
        super().__init__("robot_status")
        self.declare_parameter("battery_rate", 1.0)
        self.declare_parameter("sonar_rate", 5.0)
        self.declare_parameter("system_rate", 1.0)
        self.declare_parameter("battery_low_v", 10.8)   # 3.6 V per cell on a 3S pack
        self.declare_parameter("temp_warn_c", 75.0)
        p = lambda name: self.get_parameter(name).value

        self.host = socket.gethostname()
        self.board = Board()
        self.battery_pub = self.create_publisher(BatteryState, "battery", 10)
        self.system_pub = self.create_publisher(DiagnosticStatus, "system", 10)
        self.create_timer(1 / p("battery_rate"), self.publish_battery)
        self.create_timer(1 / p("system_rate"), self.publish_system)

        self.sonar = Sonar()
        if self.sonar.distance_mm() is not None:
            self.range_pub = self.create_publisher(Range, "sonar/range", 10)
            self.create_timer(1 / p("sonar_rate"), self.publish_range)
            self.get_logger().info("ultrasonic module found at I2C 0x77")
        else:
            self.sonar = None
            self.get_logger().info("no ultrasonic module - not publishing sonar/range")
        self.battery_v = float("nan")

    def publish_battery(self):
        msg = BatteryState()
        msg.header.stamp = self.get_clock().now().to_msg()
        mv = self.board.battery_mv()
        msg.present = mv is not None
        msg.voltage = mv / 1000 if mv is not None else float("nan")
        msg.percentage = float("nan")
        msg.current = msg.charge = msg.capacity = msg.design_capacity = float("nan")
        msg.power_supply_status = BatteryState.POWER_SUPPLY_STATUS_DISCHARGING
        msg.power_supply_technology = BatteryState.POWER_SUPPLY_TECHNOLOGY_UNKNOWN
        self.battery_v = msg.voltage
        self.battery_pub.publish(msg)

    def publish_range(self):
        mm = self.sonar.distance_mm()
        if mm is None:
            return
        msg = Range()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = "sonar_link"
        msg.radiation_type = Range.ULTRASOUND
        msg.field_of_view = math.radians(15)
        msg.min_range, msg.max_range = 0.02, 5.0
        msg.range = mm / 1000
        self.range_pub.publish(msg)

    def publish_system(self):
        temp = cpu_temp_c()
        load1 = os.getloadavg()[0]
        disk = shutil.disk_usage("/")
        flags = throttled_flags()
        uptime_s = float(_read("/proc/uptime", "0").split()[0])
        values = {
            "cpu_temp_c": f"{temp:.1f}",
            "load_1min": f"{load1:.2f}",
            "cpus": str(os.cpu_count()),
            "mem_used_percent": f"{mem_used_percent():.0f}",
            "disk_free_gb": f"{disk.free / 1e9:.1f}",
            "wifi_signal_dbm": f"{wifi_signal_dbm():.0f}",
            "ip": wlan0_ip(),
            "battery_v": f"{self.battery_v:.2f}",
            "throttled": f"0x{flags:x}" if flags is not None else "unknown",
            "uptime_s": f"{uptime_s:.0f}",
        }
        problems = []
        if flags is not None and flags & 0x1:
            problems.append("under-voltage now")
        elif flags is not None and flags & 0x10000:
            problems.append("under-voltage since boot")
        if temp >= self.get_parameter("temp_warn_c").value:
            problems.append(f"CPU {temp:.0f} C")
        if self.battery_v < self.get_parameter("battery_low_v").value:
            problems.append(f"battery low {self.battery_v:.2f} V")
        if math.isnan(self.battery_v):
            problems.append("no battery reading")

        msg = DiagnosticStatus()
        msg.name = f"{self.host}/system"
        msg.hardware_id = self.host
        msg.level = DiagnosticStatus.WARN if problems else DiagnosticStatus.OK
        msg.message = ", ".join(problems) if problems else "OK"
        msg.values = [KeyValue(key=k, value=v) for k, v in values.items()]
        self.system_pub.publish(msg)


def main():
    rclpy.init()
    node = StatusNode()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):   # Ctrl+C or systemctl stop
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == "__main__":
    main()
