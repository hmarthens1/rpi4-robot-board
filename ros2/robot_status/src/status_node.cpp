// status_node: publishes what the robot knows about itself. Read-only.
//
//   battery      sensor_msgs/BatteryState          1 Hz
//   sonar/range  sensor_msgs/Range                 5 Hz, only if the sonar answers at start-up
//   system       diagnostic_msgs/DiagnosticStatus  1 Hz: CPU temperature and load, memory,
//                disk, Wi-Fi signal, IP, battery, power flags, uptime
//
// Same topics and fields as the earlier Python node, so the laptop apps don't change.
#include <arpa/inet.h>
#include <ifaddrs.h>
#include <netinet/in.h>
#include <sys/statvfs.h>
#include <unistd.h>

#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <fstream>
#include <limits>
#include <memory>
#include <sstream>
#include <string>
#include <thread>
#include <vector>

#include "diagnostic_msgs/msg/diagnostic_status.hpp"
#include "diagnostic_msgs/msg/key_value.hpp"
#include "rclcpp/rclcpp.hpp"
#include "robot_board/board.hpp"
#include "robot_board/sonar.hpp"
#include "sensor_msgs/msg/battery_state.hpp"
#include "sensor_msgs/msg/range.hpp"

using namespace std::chrono_literals;
using diagnostic_msgs::msg::DiagnosticStatus;

namespace {
constexpr double kNaN = std::numeric_limits<double>::quiet_NaN();

std::string read_file(const std::string & path)
{
  std::ifstream f(path);
  std::stringstream ss;
  ss << f.rdbuf();
  return ss.str();
}

double cpu_temp_c()
{
  std::string s = read_file("/sys/class/thermal/thermal_zone0/temp");
  return s.empty() ? kNaN : std::stod(s) / 1000.0;
}

double mem_used_percent()
{
  std::ifstream f("/proc/meminfo");
  std::string key;
  long value = 0, total = 0, available = 0;
  std::string unit;
  while (f >> key >> value >> unit) {
    if (key == "MemTotal:") {total = value;}
    if (key == "MemAvailable:") {available = value;}
  }
  return total ? 100.0 * (1.0 - static_cast<double>(available) / total) : kNaN;
}

double wifi_signal_dbm()
{
  std::ifstream f("/proc/net/wireless");
  std::string line;
  while (std::getline(f, line)) {
    std::istringstream is(line);
    std::string iface, status, link, level;
    if (is >> iface >> status >> link >> level && iface == "wlan0:") {
      return std::stod(level);           // "-40." parses as -40
    }
  }
  return kNaN;
}

std::string wlan0_ip()
{
  ifaddrs * list = nullptr;
  std::string ip;
  if (getifaddrs(&list) == 0) {
    for (ifaddrs * a = list; a; a = a->ifa_next) {
      if (a->ifa_addr && a->ifa_addr->sa_family == AF_INET && std::string(a->ifa_name) == "wlan0") {
        char buf[INET_ADDRSTRLEN];
        inet_ntop(AF_INET, &reinterpret_cast<sockaddr_in *>(a->ifa_addr)->sin_addr, buf, sizeof buf);
        ip = buf;
      }
    }
    freeifaddrs(list);
  }
  return ip;
}

long throttled_flags()
{
  std::string s = read_file("/sys/devices/platform/soc/soc:firmware/get_throttled");
  return s.empty() ? -1 : std::strtol(s.c_str(), nullptr, 16);
}

std::string fmt(double v, int decimals)
{
  if (std::isnan(v)) {return "nan";}
  char buf[32];
  std::snprintf(buf, sizeof buf, "%.*f", decimals, v);
  return buf;
}
}  // namespace

class StatusNode : public rclcpp::Node {
public:
  StatusNode()
  : Node("robot_status")
  {
    battery_rate_ = declare_parameter("battery_rate", 1.0);
    sonar_rate_ = declare_parameter("sonar_rate", 5.0);
    system_rate_ = declare_parameter("system_rate", 1.0);
    battery_low_v_ = declare_parameter("battery_low_v", 10.8);   // 3.6 V/cell, 3S pack
    temp_warn_c_ = declare_parameter("temp_warn_c", 75.0);

    char host[256] = {};
    gethostname(host, sizeof host - 1);
    host_ = host;

    battery_pub_ = create_publisher<sensor_msgs::msg::BatteryState>("battery", 10);
    system_pub_ = create_publisher<DiagnosticStatus>("system", 10);
    timers_.push_back(create_wall_timer(period(battery_rate_), [this] {publish_battery();}));
    timers_.push_back(create_wall_timer(period(system_rate_), [this] {publish_system();}));

    try {
      sonar_ = std::make_unique<robot_board::Sonar>();
      if (!sonar_->present()) {sonar_.reset();}
    } catch (const std::exception &) {
      sonar_.reset();
    }
    if (sonar_) {
      range_pub_ = create_publisher<sensor_msgs::msg::Range>("sonar/range", 10);
      timers_.push_back(create_wall_timer(period(sonar_rate_), [this] {publish_range();}));
      RCLCPP_INFO(get_logger(), "ultrasonic module found at I2C 0x77");
    } else {
      RCLCPP_INFO(get_logger(), "no ultrasonic module - not publishing sonar/range");
    }
  }

private:
  static std::chrono::nanoseconds period(double hz)
  {
    return std::chrono::nanoseconds(static_cast<int64_t>(1e9 / hz));
  }

  void publish_battery()
  {
    sensor_msgs::msg::BatteryState msg;
    msg.header.stamp = now();
    auto mv = board_.battery_mv();
    msg.present = mv.has_value();
    msg.voltage = mv ? *mv / 1000.0f : std::numeric_limits<float>::quiet_NaN();
    msg.percentage = msg.current = msg.charge = msg.capacity = msg.design_capacity =
      std::numeric_limits<float>::quiet_NaN();
    msg.power_supply_status = sensor_msgs::msg::BatteryState::POWER_SUPPLY_STATUS_DISCHARGING;
    msg.power_supply_technology = sensor_msgs::msg::BatteryState::POWER_SUPPLY_TECHNOLOGY_UNKNOWN;
    battery_v_ = mv ? *mv / 1000.0 : kNaN;
    battery_pub_->publish(msg);
  }

  void publish_range()
  {
    auto mm = sonar_->distance_mm();
    if (!mm) {return;}
    sensor_msgs::msg::Range msg;
    msg.header.stamp = now();
    msg.header.frame_id = "sonar_link";
    msg.radiation_type = sensor_msgs::msg::Range::ULTRASOUND;
    msg.field_of_view = 15.0f * static_cast<float>(M_PI) / 180.0f;
    msg.min_range = 0.02f;
    msg.max_range = 5.0f;
    msg.range = *mm / 1000.0f;
    range_pub_->publish(msg);
  }

  void publish_system()
  {
    const double temp = cpu_temp_c();
    double load[1] = {kNaN};
    getloadavg(load, 1);
    struct statvfs disk {};
    statvfs("/", &disk);
    const double free_gb = static_cast<double>(disk.f_bavail) * disk.f_frsize / 1e9;
    const long flags = throttled_flags();
    const double uptime = std::strtod(read_file("/proc/uptime").c_str(), nullptr);
    char hex[16];
    std::snprintf(hex, sizeof hex, "0x%lx", flags);

    const std::vector<std::pair<std::string, std::string>> values = {
      {"cpu_temp_c", fmt(temp, 1)},
      {"load_1min", fmt(load[0], 2)},
      {"cpus", std::to_string(std::thread::hardware_concurrency())},
      {"mem_used_percent", fmt(mem_used_percent(), 0)},
      {"disk_free_gb", fmt(free_gb, 1)},
      {"wifi_signal_dbm", fmt(wifi_signal_dbm(), 0)},
      {"ip", wlan0_ip()},
      {"battery_v", fmt(battery_v_, 2)},
      {"throttled", flags >= 0 ? hex : "unknown"},
      {"uptime_s", fmt(uptime, 0)},
    };

    std::vector<std::string> problems;
    if (flags >= 0 && (flags & 0x1)) {
      problems.push_back("under-voltage now");
    } else if (flags >= 0 && (flags & 0x10000)) {
      problems.push_back("under-voltage since boot");
    }
    if (temp >= temp_warn_c_) {problems.push_back("CPU " + fmt(temp, 0) + " C");}
    if (std::isnan(battery_v_)) {
      problems.push_back("no battery reading");
    } else if (battery_v_ < battery_low_v_) {
      problems.push_back("battery low " + fmt(battery_v_, 2) + " V");
    }

    DiagnosticStatus msg;
    msg.name = host_ + "/system";
    msg.hardware_id = host_;
    msg.level = problems.empty() ? DiagnosticStatus::OK : DiagnosticStatus::WARN;
    for (size_t i = 0; i < problems.size(); ++i) {
      msg.message += (i ? ", " : "") + problems[i];
    }
    if (problems.empty()) {msg.message = "OK";}
    for (const auto & [k, v] : values) {
      diagnostic_msgs::msg::KeyValue kv;
      kv.key = k;
      kv.value = v;
      msg.values.push_back(kv);
    }
    system_pub_->publish(msg);
  }

  robot_board::Board board_;
  std::unique_ptr<robot_board::Sonar> sonar_;
  std::string host_;
  double battery_v_ = kNaN;
  double battery_rate_, sonar_rate_, system_rate_, battery_low_v_, temp_warn_c_;
  rclcpp::Publisher<sensor_msgs::msg::BatteryState>::SharedPtr battery_pub_;
  rclcpp::Publisher<sensor_msgs::msg::Range>::SharedPtr range_pub_;
  rclcpp::Publisher<DiagnosticStatus>::SharedPtr system_pub_;
  std::vector<rclcpp::TimerBase::SharedPtr> timers_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<StatusNode>());
  rclcpp::shutdown();
  return 0;
}
