// command_node: the robot's command listener. Turns JSON commands from the
// laptop (voice agent, dashboard, scripts) into motor, servo, LED and buzzer
// actions, with safety limits that hold whatever the sender asks for.
//
//   command         std_msgs/String      in:  one JSON command
//   command_result  std_msgs/String      out: {"id", "action", "ok", "detail"}
//   cmd_vel         geometry_msgs/Twist  in:  continuous driving; stops cmd_vel_timeout
//                                             after the last message
//
// Commands ("id" is optional and echoed back):
//   {"action": "drive", "vx": 0, "vy": 0.4, "turn": 0, "duration": 1.5}   each -1..1 of max_speed
//   {"action": "motor", "motor": 1, "speed": 0.3, "duration": 1.0}
//   {"action": "servo", "servo": 1, "pulse": 1500, "ms": 500}   or "angle": 0..180
//   {"action": "led", "led": 1, "on": true}
//   {"action": "rgb", "r": 0, "g": 0, "b": 80}                  needs root (service runs as root)
//   {"action": "buzzer", "seconds": 0.2, "times": 2}
//   {"action": "stop"}                                            always accepted
//   {"action": "status"}
//
// Safety: max_speed / max_duration / drive / cmd_vel_timeout are fixed at
// start-up (ros2 param set is refused), speeds are scaled to max_speed, each
// motion stops by itself after at most max_duration, a new motion replaces the
// running one, and the motors stop when the node exits.
#include <algorithm>
#include <atomic>
#include <chrono>
#include <memory>
#include <mutex>
#include <nlohmann/json.hpp>
#include <optional>
#include <string>
#include <thread>

#include "geometry_msgs/msg/twist.hpp"
#include "rcl_interfaces/msg/set_parameters_result.hpp"
#include "rclcpp/rclcpp.hpp"
#include "robot_board/board.hpp"
#include "robot_board/peripherals.hpp"
#include "robot_board/sonar.hpp"
#include "std_msgs/msg/string.hpp"

using json = nlohmann::json;
using namespace std::chrono_literals;
using Clock = std::chrono::steady_clock;   // monotonic: immune to clock changes

namespace {
double clip(double v, double lo = -1.0, double hi = 1.0) {return std::max(lo, std::min(hi, v));}
}

class CommandNode : public rclcpp::Node {
public:
  CommandNode()
  : Node("robot_command")
  {
    max_speed_ = declare_parameter("max_speed", 50.0);
    max_duration_ = declare_parameter("max_duration", 5.0);
    drive_ = declare_parameter("drive", std::string("mecanum"));
    cmd_vel_timeout_ = declare_parameter("cmd_vel_timeout", 0.5);
    // Limits are fixed once running: no remote client (person, script or LLM
    // agent) may lift them. Change them in the service file and restart.
    param_cb_ = add_on_set_parameters_callback([this](const std::vector<rclcpp::Parameter> & ps) {
      rcl_interfaces::msg::SetParametersResult r;
      r.successful = true;
      for (const auto & p : ps) {
        const auto & n = p.get_name();
        if (started_ && (n == "max_speed" || n == "max_duration" || n == "drive" ||
          n == "cmd_vel_timeout"))
        {
          r.successful = false;
          r.reason = n + " can only be set at startup";
        }
      }
      return r;
    });

    try {
      rgb_ = std::make_unique<robot_board::Rgb>();
    } catch (const std::exception & e) {
      RCLCPP_WARN(get_logger(), "RGB LEDs unavailable (%s) - run as root", e.what());
    }
    try {
      sonar_ = std::make_unique<robot_board::Sonar>();
      if (!sonar_->distance_mm()) {sonar_.reset();}
    } catch (const std::exception &) {
      sonar_.reset();
    }

    result_pub_ = create_publisher<std_msgs::msg::String>("command_result", 10);
    command_sub_ = create_subscription<std_msgs::msg::String>(
      "command", 10, [this](std_msgs::msg::String::ConstSharedPtr m) {on_command(m->data);});
    cmd_vel_sub_ = create_subscription<geometry_msgs::msg::Twist>(
      "cmd_vel", 10, [this](geometry_msgs::msg::Twist::ConstSharedPtr m) {on_cmd_vel(*m);});
    watchdog_ = create_wall_timer(50ms, [this] {watchdog();});

    board_.stop();
    started_ = true;
    RCLCPP_INFO(get_logger(), "ready: max_speed %.1f %%, max_duration %.1f s, drive %s, RGB %s, sonar %s",
      max_speed_, max_duration_, drive_.c_str(), rgb_ ? "yes" : "no (not root)", sonar_ ? "yes" : "no");
  }

  ~CommandNode() override
  {
    try {
      board_.stop();
      if (beep_thread_.joinable()) {beep_thread_.join();}
      if (rgb_) {rgb_->off();}
    } catch (...) {
    }
  }

private:
  // ---------------------------------------------------------------- plumbing
  void reply(const json & id, const std::string & action, bool ok, const json & detail)
  {
    std_msgs::msg::String msg;
    msg.data = json{{"id", id}, {"action", action}, {"ok", ok}, {"detail", detail}}.dump();
    result_pub_->publish(msg);
    const std::string text = detail.is_string() ? detail.get<std::string>() : detail.dump();
    if (ok) {
      RCLCPP_INFO(get_logger(), "%s: %s", action.c_str(), text.c_str());
    } else {
      RCLCPP_WARN(get_logger(), "%s: %s", action.c_str(), text.c_str());
    }
  }

  void on_command(const std::string & data)
  {
    json id = nullptr;
    std::string action = "?";
    try {
      const json cmd = json::parse(data);
      if (cmd.contains("id")) {id = cmd["id"];}
      action = cmd.value("action", "?");
      reply(id, action, true, dispatch(action, cmd));
    } catch (const std::exception & e) {   // bad JSON, bad values, I2C errors: report, never crash
      if (action != "stop") {stop_motors();}
      reply(id, action, false, std::string(e.what()));
    }
  }

  json dispatch(const std::string & action, const json & cmd)
  {
    if (action == "drive") {return do_drive(cmd);}
    if (action == "motor") {return do_motor(cmd);}
    if (action == "servo") {return do_servo(cmd);}
    if (action == "led") {return do_led(cmd);}
    if (action == "rgb") {return do_rgb(cmd);}
    if (action == "buzzer") {return do_buzzer(cmd);}
    if (action == "stop") {stop_motors(); return "all motors stopped";}
    if (action == "status") {return do_status();}
    throw std::invalid_argument("unknown action '" + action + "'");
  }

  double duration(const json & cmd) const
  {
    const double d = cmd.value("duration", 1.0);
    if (d <= 0) {throw std::invalid_argument("duration must be > 0");}
    return std::min(d, max_duration_);
  }

  json run_motion(const std::array<int, 4> & speeds, double seconds, const std::string & desc)
  {
    std::lock_guard<std::mutex> lock(motion_mutex_);
    board_.set_motors(speeds);
    motion_until_ = Clock::now() + std::chrono::duration_cast<Clock::duration>(
      std::chrono::duration<double>(seconds));
    moving_ = true;
    motion_desc_ = desc;
    char buf[128];
    std::snprintf(buf, sizeof buf, " for %.1f s (motors [%d, %d, %d, %d])", seconds,
      speeds[0], speeds[1], speeds[2], speeds[3]);
    return desc + buf;
  }

  void stop_motors()
  {
    std::lock_guard<std::mutex> lock(motion_mutex_);
    moving_ = false;
    cmd_vel_active_ = false;
    motion_desc_ = "stopped";
    board_.stop();
  }

  void watchdog()
  {
    const auto now = Clock::now();
    bool expired = false, stale = false;
    {
      std::lock_guard<std::mutex> lock(motion_mutex_);
      expired = moving_ && now >= motion_until_;
      stale = cmd_vel_active_ &&
        now - cmd_vel_last_ > std::chrono::duration<double>(cmd_vel_timeout_);
    }
    if (expired) {
      stop_motors();
      reply(nullptr, "stop", true, "motion finished");
    } else if (stale) {
      stop_motors();
    }
  }

  // ----------------------------------------------------------------- actions
  json do_drive(const json & cmd)
  {
    const double vx = clip(cmd.value("vx", 0.0)), vy = clip(cmd.value("vy", 0.0));
    const double turn = clip(cmd.value("turn", 0.0));
    if (vx != 0.0 && drive_ != "mecanum") {
      throw std::invalid_argument("this robot can't slide sideways (drive is not mecanum)");
    }
    const auto speeds = robot_board::wheel_speeds(vx * max_speed_, vy * max_speed_, turn * max_speed_);
    char desc[96];
    std::snprintf(desc, sizeof desc, "drive vx=%+.2f vy=%+.2f turn=%+.2f", vx, vy, turn);
    return run_motion(speeds, duration(cmd), desc);
  }

  json do_motor(const json & cmd)
  {
    const int motor = cmd.at("motor").get<int>();
    if (motor < 1 || motor > 4) {throw std::invalid_argument("motor must be 1-4");}
    const int speed = static_cast<int>(std::lround(clip(cmd.value("speed", 0.3)) * max_speed_));
    std::array<int, 4> speeds{};
    speeds[motor - 1] = speed;
    return run_motion(speeds, duration(cmd),
      "motor " + std::to_string(motor) + " at " + std::to_string(speed) + " %");
  }

  json do_servo(const json & cmd)
  {
    const int servo = cmd.at("servo").get<int>();
    const int ms = cmd.value("ms", 500);
    char buf[96];
    if (cmd.contains("angle")) {
      const double angle = clip(cmd["angle"].get<double>(), 0, 180);
      board_.set_servo_angle(servo, angle, ms);
      std::snprintf(buf, sizeof buf, "servo %d to %.0f deg over %d ms", servo, angle, ms);
    } else {
      const int pulse = static_cast<int>(clip(cmd.value("pulse", 1500), 500, 2500));
      board_.set_servo_pulse(servo, pulse, ms);
      std::snprintf(buf, sizeof buf, "servo %d to %d us over %d ms", servo, pulse, ms);
    }
    return std::string(buf);
  }

  json do_led(const json & cmd)
  {
    const int led = cmd.value("led", 1);
    const bool on = cmd.value("on", true);
    gpio_.led(led, on);   // its own GPIO line: never waits for the buzzer
    return "LED" + std::to_string(led) + (on ? " on" : " off");
  }

  json do_rgb(const json & cmd)
  {
    if (!rgb_) {throw std::runtime_error("RGB LEDs need the node to run as root");}
    const int r = static_cast<int>(clip(cmd.value("r", 0), 0, 255));
    const int g = static_cast<int>(clip(cmd.value("g", 0), 0, 255));
    const int b = static_cast<int>(clip(cmd.value("b", 0), 0, 255));
    rgb_->fill(r, g, b);
    return "RGB set to (" + std::to_string(r) + ", " + std::to_string(g) + ", " +
           std::to_string(b) + ")";
  }

  json do_buzzer(const json & cmd)
  {
    const double seconds = clip(cmd.value("seconds", 0.2), 0.02, 2.0);
    const int times = static_cast<int>(clip(cmd.value("times", 1), 1, 5));
    // Beep on its own thread, so the executor (and with it any stop command)
    // is never held up by a beep. A beep during a beep is refused, not queued.
    if (beeping_) {throw std::runtime_error("buzzer is busy");}
    if (beep_thread_.joinable()) {beep_thread_.join();}   // finished: returns at once
    beeping_ = true;
    beep_thread_ = std::thread([this, seconds, times] {
      try {gpio_.beep(seconds, times);} catch (...) {}
      beeping_ = false;
    });
    char buf[64];
    std::snprintf(buf, sizeof buf, "beeping %dx %.2f s", times, seconds);
    return std::string(buf);
  }

  json do_status()
  {
    std::optional<int> mm = sonar_ ? sonar_->distance_mm() : std::nullopt;
    auto v = board_.battery_v();
    json motors, servos;
    for (int i = 0; i < 4; ++i) {motors[std::to_string(i + 1)] = board_.motor_speeds()[i];}
    for (int i = 0; i < 6; ++i) {
      const auto & p = board_.servo_pulses()[i];
      servos[std::to_string(i + 1)] = p ? json(*p) : json(nullptr);
    }
    std::lock_guard<std::mutex> lock(motion_mutex_);
    return {{"battery_v", v ? json(*v) : json(nullptr)},
            {"distance_m", mm ? json(*mm / 1000.0) : json(nullptr)},
            {"motion", motion_desc_}, {"motor_speeds", motors}, {"servo_pulses", servos},
            {"max_speed", max_speed_}, {"max_duration", max_duration_}, {"drive", drive_}};
  }

  // ----------------------------------------------------------------- cmd_vel
  void on_cmd_vel(const geometry_msgs::msg::Twist & msg)
  {
    // ROS convention: +y is left; the board's mixing wants +vx = right.
    const double vx = drive_ == "mecanum" ? clip(-msg.linear.y) : 0.0;
    const double vy = clip(msg.linear.x), turn = clip(msg.angular.z);
    std::lock_guard<std::mutex> lock(motion_mutex_);
    board_.set_motors(robot_board::wheel_speeds(vx * max_speed_, vy * max_speed_, turn * max_speed_));
    moving_ = false;
    cmd_vel_active_ = true;
    cmd_vel_last_ = Clock::now();
    motion_desc_ = "cmd_vel";
  }

  robot_board::Board board_;
  robot_board::Gpio gpio_;
  std::unique_ptr<robot_board::Rgb> rgb_;
  std::unique_ptr<robot_board::Sonar> sonar_;
  std::mutex motion_mutex_;
  std::atomic<bool> beeping_{false};
  bool moving_ = false, cmd_vel_active_ = false;
  std::atomic<bool> started_{false};
  Clock::time_point motion_until_{}, cmd_vel_last_{};
  std::string motion_desc_ = "stopped";
  std::thread beep_thread_;
  double max_speed_, max_duration_, cmd_vel_timeout_;
  std::string drive_;
  OnSetParametersCallbackHandle::SharedPtr param_cb_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr result_pub_;
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr command_sub_;
  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_sub_;
  rclcpp::TimerBase::SharedPtr watchdog_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<CommandNode>());
  rclcpp::shutdown();
  return 0;
}
