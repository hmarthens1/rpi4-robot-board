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
//   {"action": "stop"}                                            always accepted (also stops an arm sequence)
//   {"action": "status"}
//
// Arm (PWM servos 1 gripper, 3 wrist, 4 elbow, 5 shoulder, 6 base; see arm.hpp):
//   {"action": "arm_pose", "pose": "stand|rest|ready|center|camera", "ms": 1500}
//   {"action": "arm_move", "x": 0, "y": 15, "z": 10, "pitch": -30, "ms": 800}   cm, deg; IK
//   {"action": "gripper", "open": true}            or "pulse": 500..2500
//   {"action": "arm_servos", "pulses": {"1": 1500, "3": 900}, "ms": 500}          raw pulses
//   {"action": "arm_sequence", "frames": [{"ms": 500, "pulses": {"1": 1500, ...}}, ...]}
//       played on its own thread, one frame after the other; stop interrupts it
//
// Safety: max_speed / max_duration / drive / cmd_vel_timeout are fixed at
// start-up (ros2 param set is refused), speeds are scaled to max_speed, each
// motion stops by itself after at most max_duration, a new motion replaces the
// running one, and the motors stop when the node exits. On a robot with the
// ultrasonic sensor, a forward move is refused when an obstacle is closer than
// min_clearance (or the distance is unknown), and stopped if it gets that close.
#include <algorithm>
#include <array>
#include <atomic>
#include <chrono>
#include <memory>
#include <mutex>
#include <nlohmann/json.hpp>
#include <map>
#include <optional>
#include <string>
#include <thread>
#include <vector>

#include "geometry_msgs/msg/twist.hpp"
#include "rcl_interfaces/msg/set_parameters_result.hpp"
#include "rclcpp/rclcpp.hpp"
#include "robot_board/arm.hpp"
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
    min_clearance_ = declare_parameter("min_clearance", 0.3);   // metres, sonar robots only
    arm_min_ms_ = declare_parameter("arm_min_ms", 300);         // no arm move faster than this
    // Per-servo calibration (us added to every pulse), servos 1..6. The defaults
    // are MasterPi's Deviation.yaml (5: -64, 6: -47); calibrate each arm.
    const auto offs = declare_parameter("servo_offsets", std::vector<int64_t>{0, 0, 0, 0, -64, -47});
    std::map<int, int> offsets;
    for (size_t i = 0; i < offs.size() && i < 6; ++i) {offsets[static_cast<int>(i) + 1] = static_cast<int>(offs[i]);}
    board_.set_servo_offsets(offsets);
    // Limits are fixed once running: no remote client (person, script or LLM
    // agent) may lift them. Change them in the service file and restart.
    param_cb_ = add_on_set_parameters_callback([this](const std::vector<rclcpp::Parameter> & ps) {
      rcl_interfaces::msg::SetParametersResult r;
      r.successful = true;
      for (const auto & p : ps) {
        const auto & n = p.get_name();
        if (started_ && (n == "max_speed" || n == "max_duration" || n == "drive" ||
          n == "cmd_vel_timeout" || n == "min_clearance" || n == "arm_min_ms"))
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
    probe_sonar();

    result_pub_ = create_publisher<std_msgs::msg::String>("command_result", 10);
    command_sub_ = create_subscription<std_msgs::msg::String>(
      "command", 10, [this](std_msgs::msg::String::ConstSharedPtr m) {on_command(m->data);});
    cmd_vel_sub_ = create_subscription<geometry_msgs::msg::Twist>(
      "cmd_vel", 10, [this](geometry_msgs::msg::Twist::ConstSharedPtr m) {on_cmd_vel(*m);});
    watchdog_ = create_wall_timer(50ms, [this] {watchdog();});
    clearance_timer_ = create_wall_timer(100ms, [this] {check_clearance();});
    // The sonar may be plugged in (or out) while the robot runs.
    sonar_probe_ = create_wall_timer(5s, [this] {probe_sonar();});

    board_.stop();
    started_ = true;
    RCLCPP_INFO(get_logger(), "ready: max_speed %.1f %%, max_duration %.1f s, drive %s, RGB %s, sonar %s",
      max_speed_, max_duration_, drive_.c_str(), rgb_ ? "yes" : "no (not root)", sonar_ ? "yes" : "no");
  }

  ~CommandNode() override
  {
    stop_sequence();
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
    if (action == "stop") {
      const bool was_playing = arm_playing_;
      stop_sequence();
      stop_motors();
      return std::string("all motors stopped") + (was_playing ? ", arm sequence stopped" : "");
    }
    if (action == "arm_pose") {return do_arm_pose(cmd);}
    if (action == "arm_move") {return do_arm_move(cmd);}
    if (action == "gripper") {return do_gripper(cmd);}
    if (action == "arm_servos") {return do_arm_servos(cmd);}
    if (action == "arm_sequence") {return do_arm_sequence(cmd);}
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
    std::lock_guard<std::recursive_mutex> lock(motion_mutex_);
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
    std::lock_guard<std::recursive_mutex> lock(motion_mutex_);
    moving_ = false;
    cmd_vel_active_ = false;
    forward_ = false;
    motion_desc_ = "stopped";
    board_.stop();
  }

  void watchdog()
  {
    const auto now = Clock::now();
    bool expired = false, stale = false;
    {
      std::lock_guard<std::recursive_mutex> lock(motion_mutex_);
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

  void probe_sonar()
  {
    if (sonar_) {return;}
    try {
      auto s = std::make_unique<robot_board::Sonar>();
      if (!s->present()) {return;}
      sonar_ = std::move(s);
    } catch (const std::exception &) {
      return;
    }
    if (started_) {
      RCLCPP_INFO(get_logger(), "ultrasonic module connected - forward obstacle check on");
    }
  }

  // A failed reading: glitch or unplugged? Only a module that no longer answers
  // at all is dropped (and then looked for again every 5 s).
  void check_unplugged()
  {
    if (sonar_ && !sonar_->present()) {
      sonar_.reset();
      RCLCPP_WARN(get_logger(), "ultrasonic module disconnected - forward obstacle check off");
    }
  }

  // Forward safety, enforced here whatever the sender (or an LLM) remembered to check.
  void require_clearance()
  {
    if (!sonar_) {return;}
    const auto mm = sonar_->distance_mm();
    if (!mm) {
      check_unplugged();
      throw std::runtime_error("refused: no valid sonar reading, path ahead unknown");
    }
    if (*mm / 1000.0 < min_clearance_) {
      char buf[96];
      std::snprintf(buf, sizeof buf, "refused: obstacle %.2f m ahead (minimum %.2f m)",
        *mm / 1000.0, min_clearance_);
      throw std::runtime_error(buf);
    }
  }

  void check_clearance()
  {
    bool forward;
    {
      std::lock_guard<std::recursive_mutex> lock(motion_mutex_);
      forward = (moving_ || cmd_vel_active_) && forward_;
    }
    if (!forward || !sonar_) {return;}
    const auto mm = sonar_->distance_mm();
    if (!mm) {check_unplugged(); return;}
    if (mm && *mm / 1000.0 < min_clearance_) {
      stop_motors();
      char buf[96];
      std::snprintf(buf, sizeof buf, "stopped: obstacle %.2f m ahead", *mm / 1000.0);
      reply(nullptr, "stop", true, std::string(buf));
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
    if (vy > 0) {require_clearance();}
    const auto speeds = robot_board::wheel_speeds(vx * max_speed_, vy * max_speed_, turn * max_speed_);
    char desc[96];
    std::snprintf(desc, sizeof desc, "drive vx=%+.2f vy=%+.2f turn=%+.2f", vx, vy, turn);
    json result = run_motion(speeds, duration(cmd), desc);
    {
      std::lock_guard<std::recursive_mutex> lock(motion_mutex_);
      forward_ = vy > 0;
    }
    return result;
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
    if (servo != 2) {require_arm_idle();}
    int pulse;
    if (cmd.contains("angle")) {
      const double angle = clip(cmd["angle"].get<double>(), 0, 180);
      pulse = static_cast<int>(500 + angle * 2000.0 / 180.0);
      std::snprintf(buf, sizeof buf, "servo %d to %.0f deg over %d ms", servo, angle, ms);
    } else {
      pulse = static_cast<int>(clip(cmd.value("pulse", 1500), 500, 2500));
      std::snprintf(buf, sizeof buf, "servo %d to %d us over %d ms", servo, pulse, ms);
    }
    {
      std::lock_guard<std::recursive_mutex> lock(motion_mutex_);
      board_.set_servo_pulse(servo, pulse, ms);
      if (servo >= 1 && servo <= 6) {arm_raw_[servo - 1] = pulse;}
    }
    return std::string(buf);
  }

  // ---------------------------------------------------------------- arm
  // Raw (uncalibrated) pulses last commanded, servos 1..6; 0 = never moved.
  // The servos give no feedback, so this is the only record of where they are.
  int raw(int servo) const {return arm_raw_[servo - 1] ? arm_raw_[servo - 1] : 1500;}

  // Move several arm servos together (one I2C frame, same time for all).
  void arm_write(const std::map<int, int> & pulses, int ms)
  {
    std::map<int, int> clipped;
    for (const auto & [servo, p] : pulses) {
      if (servo < 1 || servo > 6 || servo == 2) {
        throw std::invalid_argument("arm servos are 1, 3, 4, 5, 6 (2 is the fan port)");
      }
      clipped[servo] = std::max(500, std::min(2500, p));
    }
    std::lock_guard<std::recursive_mutex> lock(motion_mutex_);
    board_.set_servo_pulses(clipped, ms);
    for (const auto & [servo, p] : clipped) {arm_raw_[servo - 1] = p;}
  }

  // Move time: as asked, or ArmIK's rule (1 ms per us of the largest change),
  // never faster than arm_min_ms, never longer than 30 s.
  int arm_ms(const json & cmd, const std::map<int, int> & target) const
  {
    int ms = cmd.value("ms", -1);
    if (ms < 0) {
      for (const auto & [servo, p] : target) {ms = std::max(ms, std::abs(p - raw(servo)));}
    }
    return std::max(static_cast<int>(arm_min_ms_), std::min(ms, 30000));
  }

  void require_arm_idle() const
  {
    if (arm_playing_) {throw std::runtime_error("arm sequence running - send stop first");}
  }

  static std::string describe(const std::map<int, int> & p, int ms)
  {
    std::string out = "servos";
    for (const auto & [servo, v] : p) {out += " " + std::to_string(servo) + "=" + std::to_string(v);}
    return out + " over " + std::to_string(ms) + " ms";
  }

  json do_arm_pose(const json & cmd)
  {
    require_arm_idle();
    const std::string name = cmd.value("pose", "stand");
    std::map<int, int> target;
    if (name == "camera") {   // the camera-view pose used by the MasterPi functions
      auto sol = ik_.solve(0, 6, 18, 0, -90, 90);
      if (!sol) {throw std::runtime_error("camera pose unreachable");}
      target = {{3, sol->servo3}, {4, sol->servo4}, {5, sol->servo5}, {6, sol->servo6}};
    } else {
      const auto & poses = robot_board::arm_poses();
      auto it = poses.find(name);
      if (it == poses.end()) {
        throw std::invalid_argument("unknown pose '" + name + "' (stand, rest, ready, center, camera)");
      }
      const auto & q = it->second;
      target = {{1, q.s1}, {3, q.s3}, {4, q.s4}, {5, q.s5}, {6, q.s6}};
    }
    const int ms = arm_ms(cmd, target);
    arm_write(target, ms);
    return "arm to '" + name + "': " + describe(target, ms);
  }

  json do_arm_move(const json & cmd)
  {
    require_arm_idle();
    const double x = cmd.at("x").get<double>(), y = cmd.at("y").get<double>();
    const double z = cmd.at("z").get<double>();
    const double pitch = cmd.value("pitch", 0.0);
    const double lo = cmd.value("pitch_min", -90.0), hi = cmd.value("pitch_max", 90.0);
    auto sol = ik_.solve(x, y, z, pitch, lo, hi);
    char where[96];
    std::snprintf(where, sizeof where, "(%.1f, %.1f, %.1f) cm", x, y, z);
    if (!sol) {
      throw std::runtime_error(std::string("unreachable: ") + where +
                               " (reach is about 22 cm from the base, joints +-90 deg)");
    }
    const std::map<int, int> target = {{3, sol->servo3}, {4, sol->servo4}, {5, sol->servo5},
                                       {6, sol->servo6}};
    const int ms = arm_ms(cmd, target);
    arm_write(target, ms);
    char buf[64];
    std::snprintf(buf, sizeof buf, " at pitch %.0f deg: ", sol->alpha);
    return std::string("arm to ") + where + buf + describe(target, ms);
  }

  json do_gripper(const json & cmd)
  {
    require_arm_idle();
    int pulse;
    if (cmd.contains("pulse")) {
      pulse = cmd["pulse"].get<int>();
    } else {
      pulse = cmd.value("open", true) ? robot_board::kGripperOpen : robot_board::kGripperClosed;
    }
    const std::map<int, int> target = {{1, pulse}};
    const int ms = arm_ms(cmd, target);
    arm_write(target, ms);
    return std::string("gripper ") + (cmd.contains("pulse") ? "to " + std::to_string(pulse) :
           (pulse == robot_board::kGripperOpen ? "open" : "closed")) + " over " + std::to_string(ms) + " ms";
  }

  json do_arm_servos(const json & cmd)
  {
    require_arm_idle();
    std::map<int, int> target;
    for (const auto & [k, v] : cmd.at("pulses").items()) {target[std::stoi(k)] = v.get<int>();}
    if (target.empty()) {throw std::invalid_argument("pulses is empty");}
    const int ms = arm_ms(cmd, target);
    arm_write(target, ms);
    return describe(target, ms);
  }

  json do_arm_sequence(const json & cmd)
  {
    require_arm_idle();
    struct Frame {std::map<int, int> pulses; int ms;};
    std::vector<Frame> frames;
    for (const auto & f : cmd.at("frames")) {
      Frame fr;
      for (const auto & [k, v] : f.at("pulses").items()) {fr.pulses[std::stoi(k)] = v.get<int>();}
      fr.ms = std::max(20, std::min(f.value("ms", 500), 30000));   // .d6a frames can be 100 ms
      frames.push_back(fr);
    }
    if (frames.empty() || frames.size() > 500) {throw std::invalid_argument("1-500 frames");}
    const std::string name = cmd.value("name", "sequence");
    if (seq_thread_.joinable()) {seq_thread_.join();}   // finished: returns at once
    arm_stop_ = false;
    arm_playing_ = true;
    seq_thread_ = std::thread([this, frames, name] {
      size_t done = 0;
      std::string error;
      for (const auto & f : frames) {
        if (arm_stop_) {break;}
        try {
          arm_write(f.pulses, f.ms);
        } catch (const std::exception & e) {
          error = e.what();
          break;
        }
        ++done;
        // wait for the frame to finish, but notice a stop within 20 ms
        for (int t = 0; t < f.ms && !arm_stop_; t += 20) {
          std::this_thread::sleep_for(std::chrono::milliseconds(20));
        }
      }
      arm_playing_ = false;
      const bool ok = error.empty() && !arm_stop_;
      reply(nullptr, "arm_sequence", ok || arm_stop_,
        "'" + name + "' " + (arm_stop_ ? "stopped" : (error.empty() ? "finished" : "failed: " + error)) +
        " after " + std::to_string(done) + "/" + std::to_string(frames.size()) + " frames");
    });
    return "playing '" + name + "': " + std::to_string(frames.size()) + " frames";
  }

  void stop_sequence()
  {
    arm_stop_ = true;
    if (seq_thread_.joinable()) {seq_thread_.join();}
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

  json arm_json() const
  {
    json a;
    for (int s : {1, 3, 4, 5, 6}) {
      a[std::to_string(s)] = arm_raw_[s - 1] ? json(arm_raw_[s - 1]) : json(nullptr);
    }
    return a;
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
    std::lock_guard<std::recursive_mutex> lock(motion_mutex_);
    return {{"battery_v", v ? json(*v) : json(nullptr)},
            {"distance_m", mm ? json(*mm / 1000.0) : json(nullptr)},
            {"motion", motion_desc_}, {"motor_speeds", motors}, {"servo_pulses", servos},
            {"max_speed", max_speed_}, {"max_duration", max_duration_}, {"drive", drive_},
            {"min_clearance_m", sonar_ ? json(min_clearance_) : json(nullptr)},
            {"arm_raw_pulses", arm_json()}, {"arm_sequence_running", arm_playing_.load()}};
  }

  // ----------------------------------------------------------------- cmd_vel
  void on_cmd_vel(const geometry_msgs::msg::Twist & msg)
  {
    // ROS convention: +y is left; the board's mixing wants +vx = right.
    const double vx = drive_ == "mecanum" ? clip(-msg.linear.y) : 0.0;
    const double vy = clip(msg.linear.x), turn = clip(msg.angular.z);
    std::lock_guard<std::recursive_mutex> lock(motion_mutex_);
    board_.set_motors(robot_board::wheel_speeds(vx * max_speed_, vy * max_speed_, turn * max_speed_));
    forward_ = vy > 0;       // check_clearance() stops it if an obstacle gets close
    moving_ = false;
    cmd_vel_active_ = true;
    cmd_vel_last_ = Clock::now();
    motion_desc_ = "cmd_vel";
  }

  robot_board::Board board_;
  robot_board::Gpio gpio_;
  std::unique_ptr<robot_board::Rgb> rgb_;
  std::unique_ptr<robot_board::Sonar> sonar_;
  std::recursive_mutex motion_mutex_;   // every board_ access, from any thread
  std::atomic<bool> beeping_{false};
  bool moving_ = false, cmd_vel_active_ = false, forward_ = false;
  std::atomic<bool> started_{false};
  Clock::time_point motion_until_{}, cmd_vel_last_{};
  std::string motion_desc_ = "stopped";
  std::thread beep_thread_;
  robot_board::ArmIK ik_;
  std::array<int, 6> arm_raw_{};
  std::thread seq_thread_;
  std::atomic<bool> arm_playing_{false}, arm_stop_{false};
  int64_t arm_min_ms_ = 300;
  double max_speed_, max_duration_, cmd_vel_timeout_, min_clearance_;
  std::string drive_;
  OnSetParametersCallbackHandle::SharedPtr param_cb_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr result_pub_;
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr command_sub_;
  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_sub_;
  rclcpp::TimerBase::SharedPtr watchdog_, clearance_timer_, sonar_probe_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<CommandNode>());
  rclcpp::shutdown();
  return 0;
}
