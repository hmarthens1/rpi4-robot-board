// vision_node: the robot's USB camera, colour lane tracking and YOLOX detection.
//
//   vision/lane              std_msgs/String  out: {"found", "offset", "angle_deg", "bands", "coverage", "fps"}
//   vision/detections        std_msgs/String  out: {"objects": [{"name", "prob", "x", "y", "w", "h"}], "ms", "model"}
//   vision/image/compressed  sensor_msgs/CompressedImage  out: JPEG with the overlays (only while subscribed)
//   vision/state             std_msgs/String  out, 1 Hz: camera, rates and the current settings
//   vision/control           std_msgs/String  in: JSON, any of
//       {"lane": true, "color": "yellow", "hsv": [h, s, v, h, s, v], "roi_top": 0.5, "two_lines": false,
//        "detect": true, "model": "nano|tiny", "threshold": 0.4, "classes": ["cup", "person"],
//        "follow": true, "search": true, "look": {"4": 2093, "5": 1826}, "speed": 0.7, "min_drive": 0.6,
//        "kp": 0.8, "ka": 0.5, "image": "annotated|mask|raw|off"}
//   cmd_vel                  geometry_msgs/Twist  out: only while searching for or following a lane
//   command                  std_msgs/String  in: listens for {"action": "stop"} (stops following)
//                                             out: arm_servos to pan the camera while searching
//
// The camera (the first USB camera, or the `camera` parameter) is read on its own
// thread and may be plugged in or out while the node runs. Lane tracking runs on
// every frame, YOLOX on its own thread as fast as it can (a few frames/s).
//
// Lane following publishes cmd_vel; command_node on the same robot still applies
// its limits (speed cap, obstacle stop, 0.5 s cmd_vel timeout). Following is off
// at start and stops on a "stop" command. With "search" (default) the camera on
// the arm is first put in the `look` pose and panned with the base servo to find
// the lane, the robot turns to face it, follows it, and searches again when it
// is lost for `lost_timeout` s (robot_vision/lane_seeker.hpp). Without, it only
// follows a lane already in view and stops when it is lost.
#include <fcntl.h>
#include <linux/videodev2.h>
#include <sys/ioctl.h>
#include <unistd.h>

#include <algorithm>
#include <atomic>
#include <chrono>
#include <cmath>
#include <condition_variable>
#include <filesystem>
#include <memory>
#include <mutex>
#include <set>
#include <string>
#include <thread>
#include <vector>

#include <nlohmann/json.hpp>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/videoio.hpp>

#include "geometry_msgs/msg/twist.hpp"
#include "rclcpp/rclcpp.hpp"
#include "robot_vision/lane.hpp"
#include "robot_vision/lane_seeker.hpp"
#include "robot_vision/yolox.hpp"
#include "sensor_msgs/msg/compressed_image.hpp"
#include "std_msgs/msg/string.hpp"

using json = nlohmann::json;
using namespace std::chrono_literals;
using Clock = std::chrono::steady_clock;

namespace
{
// First /dev/videoN that is a UVC (USB) camera able to capture: skips the Pi's
// own codec devices and the cameras' metadata nodes.
std::string find_usb_camera()
{
  std::vector<std::string> devs;
  for (const auto & e : std::filesystem::directory_iterator("/dev")) {
    const auto name = e.path().filename().string();
    if (name.rfind("video", 0) == 0) {devs.push_back(e.path().string());}
  }
  std::sort(devs.begin(), devs.end(), [](const auto & a, const auto & b) {
      return std::stoi(a.substr(10)) < std::stoi(b.substr(10));});
  for (const auto & dev : devs) {
    const int fd = ::open(dev.c_str(), O_RDWR | O_NONBLOCK);
    if (fd < 0) {continue;}
    v4l2_capability cap{};
    const bool ok = ::ioctl(fd, VIDIOC_QUERYCAP, &cap) == 0 &&
      std::string(reinterpret_cast<const char *>(cap.driver)) == "uvcvideo" &&
      (cap.device_caps & V4L2_CAP_VIDEO_CAPTURE);
    ::close(fd);
    if (ok) {return dev;}
  }
  return "";
}

double seconds_since(Clock::time_point t)
{
  return std::chrono::duration<double>(Clock::now() - t).count();
}

double now_s()
{
  return std::chrono::duration<double>(Clock::now().time_since_epoch()).count();
}
}  // namespace

class VisionNode : public rclcpp::Node
{
public:
  VisionNode()
  : Node("vision")
  {
    camera_param_ = declare_parameter("camera", std::string("auto"));
    cam_w_ = declare_parameter("width", 640);
    cam_h_ = declare_parameter("height", 480);
    models_dir_ = declare_parameter("models_dir", std::string("/usr/local/share/robot_vision/models"));
    threads_ = declare_parameter("detect_threads", 3);
    image_rate_ = declare_parameter("image_rate", 3.0);
    image_width_ = declare_parameter("image_width", 320);
    seeker_.config().lost_timeout = declare_parameter("lost_timeout", 1.0);
    search_ = declare_parameter("search", true);
    // The arm pose that shows the floor ahead (robot01, 2026-09-28: elbow and
    // shoulder only; the base servo pans). JSON {"servo": pulse}.
    look_ = json::parse(declare_parameter("look", std::string(R"({"4": 2093, "5": 1826})")));

    lane_on_ = declare_parameter("lane", true);
    std::string color = declare_parameter("lane_color", std::string("yellow"));
    if (!robot_vision::color_preset(color, lane_cfg_)) {
      RCLCPP_WARN(get_logger(), "unknown lane_color '%s', using yellow", color.c_str());
      robot_vision::color_preset(color = "yellow", lane_cfg_);
    }
    color_ = color;
    lane_cfg_.roi_top = declare_parameter("roi_top", 0.5);
    detect_on_ = declare_parameter("detect", false);
    model_ = declare_parameter("model", std::string("nano"));
    threshold_ = declare_parameter("threshold", 0.4);

    lane_pub_ = create_publisher<std_msgs::msg::String>("vision/lane", 10);
    det_pub_ = create_publisher<std_msgs::msg::String>("vision/detections", 10);
    state_pub_ = create_publisher<std_msgs::msg::String>("vision/state", 10);
    image_pub_ = create_publisher<sensor_msgs::msg::CompressedImage>(
      "vision/image/compressed", rclcpp::SensorDataQoS().keep_last(1));
    cmd_vel_pub_ = create_publisher<geometry_msgs::msg::Twist>("cmd_vel", 10);
    arm_pub_ = create_publisher<std_msgs::msg::String>("command", 10);
    control_sub_ = create_subscription<std_msgs::msg::String>("vision/control", 10,
      [this](const std_msgs::msg::String & m) {on_control(m.data);});
    command_sub_ = create_subscription<std_msgs::msg::String>("command", 10,
      [this](const std_msgs::msg::String & m) {
        try {
          if (json::parse(m.data).value("action", "") == "stop") {set_follow(false, "stop command");}
        } catch (const std::exception &) {}
      });
    state_timer_ = create_wall_timer(1s, [this] {publish_state(true);});

    running_ = true;
    capture_thread_ = std::thread([this] {capture_loop();});
    process_thread_ = std::thread([this] {process_loop();});
    detect_thread_ = std::thread([this] {detect_loop();});
    RCLCPP_INFO(get_logger(), "ready: lane %s (%s), detect %s (%s), models in %s",
      lane_on_ ? "on" : "off", color_.c_str(), detect_on_ ? "on" : "off", model_.c_str(),
      models_dir_.c_str());
  }

  ~VisionNode() override
  {
    running_ = false;
    frame_cv_.notify_all();
    for (auto * t : {&capture_thread_, &process_thread_, &detect_thread_}) {
      if (t->joinable()) {t->join();}
    }
    if (following_) {cmd_vel_pub_->publish(geometry_msgs::msg::Twist());}   // (no locks: threads are gone)
  }

private:
  // ---------------------------------------------------------------- camera
  void capture_loop()
  {
    cv::VideoCapture cap;
    cv::Mat frame;
    int failures = 0;
    while (running_) {
      if (!cap.isOpened()) {
        const std::string dev = camera_param_ == "auto" ? find_usb_camera() : camera_param_;
        if (!dev.empty() && cap.open(dev, cv::CAP_V4L2)) {
          cap.set(cv::CAP_PROP_FRAME_WIDTH, cam_w_);
          cap.set(cv::CAP_PROP_FRAME_HEIGHT, cam_h_);
          cap.set(cv::CAP_PROP_BUFFERSIZE, 1);
          RCLCPP_INFO(get_logger(), "camera %s open: %dx%d", dev.c_str(),
            static_cast<int>(cap.get(cv::CAP_PROP_FRAME_WIDTH)),
            static_cast<int>(cap.get(cv::CAP_PROP_FRAME_HEIGHT)));
          {
            std::lock_guard<std::mutex> lock(status_mutex_);
            camera_dev_ = dev;
          }
          failures = 0;
        } else {
          camera_ok_ = false;
          for (int i = 0; i < 50 && running_; ++i) {std::this_thread::sleep_for(100ms);}   // look again in 5 s
          continue;
        }
      }
      if (!cap.read(frame) || frame.empty()) {
        if (++failures >= 10) {
          RCLCPP_WARN(get_logger(), "camera stopped answering - unplugged?");
          cap.release();
          camera_ok_ = false;
        }
        std::this_thread::sleep_for(50ms);
        continue;
      }
      failures = 0;
      camera_ok_ = true;
      {
        std::lock_guard<std::mutex> lock(frame_mutex_);
        frame.copyTo(frame_);
        ++frame_id_;
      }
      frame_cv_.notify_all();
      capture_count_++;
    }
  }

  void set_event(const std::string & e)
  {
    std::lock_guard<std::mutex> lock(status_mutex_);
    event_ = e;
  }

  bool wait_frame(uint64_t & seen, cv::Mat & out)
  {
    std::unique_lock<std::mutex> lock(frame_mutex_);
    frame_cv_.wait_for(lock, 500ms, [&] {return !running_ || frame_id_ != seen;});
    if (!running_ || frame_id_ == seen) {return false;}
    seen = frame_id_;
    frame_.copyTo(out);
    return true;
  }

  // ------------------------------------------------------ lane + image
  void process_loop()
  {
    uint64_t seen = 0;
    cv::Mat frame;
    auto last_image = Clock::now();
    while (running_) {
      if (!wait_frame(seen, frame)) {
        if (following_ && !camera_ok_) {set_follow(false, "camera lost");}
        continue;
      }
      robot_vision::LaneResult lane;
      robot_vision::LaneConfig cfg;
      bool lane_on;
      std::string image_mode;
      {
        std::lock_guard<std::mutex> lock(settings_mutex_);
        cfg = lane_cfg_;
        lane_on = lane_on_;
        image_mode = image_mode_;
      }
      if (lane_on) {
        lane = robot_vision::detect_lane(frame, cfg);
        publish_lane(lane);
        follow(lane);
      }
      const bool want_image = image_pub_->get_subscription_count() > 0 && image_mode != "off";
      if (want_image && seconds_since(last_image) >= 1.0 / std::max(0.1, image_rate_)) {
        last_image = Clock::now();
        publish_image(frame, lane, cfg, lane_on, image_mode);
      }
    }
  }

  void publish_lane(const robot_vision::LaneResult & lane)
  {
    const double dt = seconds_since(last_lane_time_);
    last_lane_time_ = Clock::now();
    lane_fps_ = 0.8 * lane_fps_.load() + 0.2 * (dt > 0 ? 1.0 / dt : 0.0);
    std_msgs::msg::String msg;
    msg.data = json{{"found", lane.found}, {"offset", std::round(lane.offset * 1000) / 1000},
      {"angle_deg", std::round(lane.angle * 1800 / M_PI) / 10}, {"bands", lane.bands_found},
      {"coverage", std::round(lane.coverage * 1000) / 1000}, {"fps", std::round(lane_fps_ * 10) / 10}}.dump();
    lane_pub_->publish(msg);
  }

  void publish_image(const cv::Mat & frame, const robot_vision::LaneResult & lane,
    const robot_vision::LaneConfig & cfg, bool lane_on, const std::string & image_mode)
  {
    cv::Mat img;
    if (image_mode == "mask" && lane_on && !lane.mask.empty()) {
      cv::cvtColor(lane.mask, img, cv::COLOR_GRAY2BGR);
    } else {
      img = frame.clone();
      if (image_mode != "raw") {
        if (lane_on) {robot_vision::draw_lane(img, lane, cfg);}
        std::lock_guard<std::mutex> lock(det_mutex_);
        if (detect_on_ && seconds_since(dets_time_) < 2.0) {robot_vision::draw_detections(img, dets_);}
      }
    }
    if (img.cols > image_width_) {
      cv::resize(img, img, cv::Size(), static_cast<double>(image_width_) / img.cols,
        static_cast<double>(image_width_) / img.cols, cv::INTER_AREA);
    }
    sensor_msgs::msg::CompressedImage msg;
    msg.header.stamp = now();
    msg.header.frame_id = "camera";
    msg.format = "jpeg";
    cv::imencode(".jpg", img, msg.data, {cv::IMWRITE_JPEG_QUALITY, 70});
    image_pub_->publish(msg);
  }

  // ------------------------------------------------------- lane following
  void follow(const robot_vision::LaneResult & lane)
  {
    if (!following_) {return;}
    std::lock_guard<std::mutex> lock(seek_mutex_);
    apply(seeker_.update(now_s(), lane));
  }

  // Does what the seeker asks. Called with seek_mutex_ held.
  void apply(const robot_vision::SeekStep & step, const json & extra_pulses = json::object())
  {
    json pulses = extra_pulses;
    if (step.base) {pulses["6"] = *step.base;}
    if (!pulses.empty()) {
      std_msgs::msg::String m;
      m.data = json{{"action", "arm_servos"}, {"pulses", pulses},
        {"ms", extra_pulses.empty() ? 400 : 800}, {"id", "lane-search"}}.dump();
      arm_pub_->publish(m);
    }
    if (step.drive) {
      geometry_msgs::msg::Twist t;
      t.linear.x = step.forward;
      t.angular.z = step.turn;
      cmd_vel_pub_->publish(t);
    }
    if (!step.event.empty()) {
      RCLCPP_INFO(get_logger(), "lane: %s", step.event.c_str());
      set_event("lane: " + step.event);
    }
    following_ = seeker_.active();
    phase_ = robot_vision::to_string(seeker_.phase());
  }

  void set_follow(bool on, const std::string & why)
  {
    std::lock_guard<std::mutex> lock(seek_mutex_);
    if (!on) {
      if (seeker_.active()) {apply(seeker_.stop(why));}
      return;
    }
    bool search;
    json look;
    {
      std::lock_guard<std::mutex> settings(settings_mutex_);
      auto & c = seeker_.config();
      c.speed = speed_;
      c.min_drive = min_drive_;
      c.kp = kp_;
      c.ka = ka_;
      search = search_;
      look = look_;
    }
    // The look pose goes out with the first pan, in one command.
    apply(seeker_.start(now_s(), search), search && look.is_object() ? look : json::object());
  }

  // ------------------------------------------------------------ detection
  void detect_loop()
  {
    robot_vision::Yolox yolo;
    std::string loaded_model;
    uint64_t seen = 0;
    cv::Mat frame;
    while (running_) {
      if (!detect_on_) {
        std::this_thread::sleep_for(200ms);
        continue;
      }
      std::string model;
      float threshold;
      std::set<std::string> classes;
      {
        std::lock_guard<std::mutex> lock(settings_mutex_);
        model = model_;
        threshold = static_cast<float>(threshold_);
        classes = classes_;
      }
      if (model != loaded_model) {
        const std::string base = models_dir_ + (model == "tiny" ? "/yoloxT" : "/yoloxN");
        if (!yolo.load(base + ".param", base + ".bin", static_cast<int>(threads_))) {
          RCLCPP_ERROR(get_logger(), "can't load %s.param/.bin (sudo bash scripts/install_vision.sh)",
            base.c_str());
          detect_on_ = false;
          set_event("detect off: model " + model + " missing");
          continue;
        }
        loaded_model = model;
        RCLCPP_INFO(get_logger(), "YOLOX %s loaded (%ld threads)", model.c_str(), threads_);
      }
      if (!wait_frame(seen, frame)) {continue;}
      const auto t0 = Clock::now();
      auto dets = yolo.detect(frame, threshold);
      const double ms = seconds_since(t0) * 1000;
      if (!classes.empty()) {
        dets.erase(std::remove_if(dets.begin(), dets.end(),
          [&](const auto & d) {return !classes.count(d.name);}), dets.end());
      }
      json objects = json::array();
      for (const auto & d : dets) {
        objects.push_back({{"name", d.name}, {"prob", std::round(d.prob * 100) / 100},
          {"x", std::lround(d.box.x)}, {"y", std::lround(d.box.y)},
          {"w", std::lround(d.box.width)}, {"h", std::lround(d.box.height)}});
      }
      std_msgs::msg::String msg;
      msg.data = json{{"objects", objects}, {"ms", std::lround(ms)}, {"model", loaded_model},
        {"image_w", frame.cols}, {"image_h", frame.rows}}.dump();
      det_pub_->publish(msg);
      detect_ms_ = 0.8 * detect_ms_.load() + 0.2 * ms;
      std::lock_guard<std::mutex> lock(det_mutex_);
      dets_ = std::move(dets);
      dets_time_ = Clock::now();
    }
  }

  // --------------------------------------------------------------- control
  void on_control(const std::string & data)
  {
    follow_request_ = -1;
    try {
      const json c = json::parse(data);
      std::lock_guard<std::mutex> lock(settings_mutex_);
      if (c.contains("color")) {
        const std::string name = c["color"];
        if (!robot_vision::color_preset(name, lane_cfg_.ranges)) {
          throw std::invalid_argument("unknown color '" + name + "'");
        }
        color_ = name;
      }
      if (c.contains("hsv")) {
        const auto v = c["hsv"].get<std::vector<double>>();
        if (v.size() != 6) {throw std::invalid_argument("hsv needs [h, s, v, h, s, v]");}
        lane_cfg_.ranges = {{{v[0], v[1], v[2]}, {v[3], v[4], v[5]}}};
        color_ = "custom";
      }
      if (c.contains("roi_top")) {lane_cfg_.roi_top = std::clamp(c["roi_top"].get<double>(), 0.0, 0.9);}
      if (c.contains("two_lines")) {lane_cfg_.two_lines = c["two_lines"].get<bool>();}
      if (c.contains("lane")) {lane_on_ = c["lane"].get<bool>();}
      if (c.contains("model")) {
        const std::string m = c["model"];
        if (m != "nano" && m != "tiny") {throw std::invalid_argument("model is nano or tiny");}
        model_ = m;
      }
      if (c.contains("threshold")) {threshold_ = std::clamp(c["threshold"].get<double>(), 0.05, 0.95);}
      if (c.contains("classes")) {
        classes_.clear();
        for (const auto & n : c["classes"]) {classes_.insert(n.get<std::string>());}
      }
      if (c.contains("detect")) {detect_on_ = c["detect"].get<bool>();}
      if (c.contains("speed")) {speed_ = std::clamp(c["speed"].get<double>(), 0.0, 1.0);}
      if (c.contains("min_drive")) {min_drive_ = std::clamp(c["min_drive"].get<double>(), 0.0, 1.0);}
      if (c.contains("search")) {search_ = c["search"].get<bool>();}
      if (c.contains("look")) {
        const auto & l = c["look"];
        if (!l.is_null() && !l.is_boolean() && !l.is_object()) {
          throw std::invalid_argument("look is {\"servo\": pulse, ...}, or false to leave the arm");
        }
        if (l.is_object()) {
          for (const auto & [k, v] : l.items()) {
            const int servo = std::stoi(k), pulse = v.get<int>();
            if (servo != 3 && servo != 4 && servo != 5) {throw std::invalid_argument("look sets servos 3, 4, 5");}
            if (pulse < 500 || pulse > 2500) {throw std::invalid_argument("look pulses are 500..2500");}
          }
        }
        look_ = l.is_object() ? l : json();
      }
      if (c.contains("kp")) {kp_ = std::clamp(c["kp"].get<double>(), 0.0, 3.0);}
      if (c.contains("ka")) {ka_ = std::clamp(c["ka"].get<double>(), 0.0, 3.0);}
      if (c.contains("image")) {
        const std::string m = c["image"];
        if (m != "annotated" && m != "mask" && m != "raw" && m != "off") {
          throw std::invalid_argument("image is annotated, mask, raw or off");
        }
        image_mode_ = m;
      }
      if (c.contains("follow")) {
        const bool on = c["follow"].get<bool>();
        if (on && !lane_on_) {throw std::invalid_argument("turn lane tracking on before following");}
        if (on && !camera_ok_) {throw std::invalid_argument("no camera");}
        follow_request_ = on ? 1 : 0;
      } else {
        set_event("settings changed");
      }
    } catch (const std::exception & e) {
      set_event(std::string("control refused: ") + e.what());
      RCLCPP_WARN(get_logger(), "control refused: %s", e.what());
    }
    // After settings_mutex_ is released (set_follow takes seek_mutex_, then settings_mutex_).
    if (follow_request_ >= 0) {set_follow(follow_request_ == 1, "asked");}
    publish_state(false);
  }

  void publish_state(bool from_timer)
  {
    if (from_timer) {
      const auto captured = capture_count_.exchange(0);
      const double dt = seconds_since(last_state_);
      last_state_ = Clock::now();
      camera_fps_ = std::round(captured / std::max(dt, 0.1) * 10) / 10;
    }
    std::string camera, event;
    {
      std::lock_guard<std::mutex> lock(status_mutex_);
      camera = camera_ok_ ? camera_dev_ : "";
      event = event_;
    }
    json s;
    {
      std::lock_guard<std::mutex> lock(settings_mutex_);
      s = {{"camera", camera}, {"camera_fps", camera_fps_},
        {"lane", lane_on_.load()}, {"color", color_}, {"roi_top", lane_cfg_.roi_top},
        {"two_lines", lane_cfg_.two_lines}, {"lane_fps", std::round(lane_fps_.load() * 10) / 10},
        {"detect", detect_on_.load()}, {"model", model_}, {"threshold", threshold_},
        {"classes", classes_}, {"detect_ms", std::lround(detect_ms_.load())},
        {"follow", following_.load()}, {"phase", phase_.load()}, {"search", search_}, {"look", look_},
        {"speed", speed_}, {"min_drive", min_drive_}, {"kp", kp_}, {"ka", ka_},
        {"image", image_mode_}, {"colors", robot_vision::color_presets()}, {"event", event}};
    }
    std_msgs::msg::String msg;
    msg.data = s.dump();
    state_pub_->publish(msg);
  }

  // parameters
  std::string camera_param_, models_dir_;
  int64_t cam_w_, cam_h_, threads_, image_width_;
  double image_rate_, camera_fps_ = 0.0;
  // settings (vision/control)
  std::mutex settings_mutex_;
  robot_vision::LaneConfig lane_cfg_;
  std::string color_, model_, image_mode_ = "annotated";
  std::string event_;   // under status_mutex_
  std::atomic<bool> lane_on_{true}, detect_on_{false}, following_{false};
  double threshold_ = 0.4, speed_ = 0.7, min_drive_ = 0.6, kp_ = 0.8, ka_ = 0.5;
  bool search_ = true;
  json look_;
  int follow_request_ = -1;          // on_control only: -1 none, 0 stop, 1 start
  // search + follow (seek_mutex_ before settings_mutex_)
  std::mutex seek_mutex_;
  robot_vision::LaneSeeker seeker_;
  std::atomic<const char *> phase_{"idle"};
  std::set<std::string> classes_;
  // camera
  std::string camera_dev_;
  std::atomic<bool> camera_ok_{false}, running_{false};
  std::mutex frame_mutex_;
  std::condition_variable frame_cv_;
  cv::Mat frame_;
  uint64_t frame_id_ = 0;
  std::atomic<int> capture_count_{0};
  // results
  std::mutex det_mutex_;
  std::vector<robot_vision::Detection> dets_;
  Clock::time_point dets_time_{}, last_lane_time_ = Clock::now(), last_state_ = Clock::now();
  std::atomic<double> detect_ms_{0.0}, lane_fps_{0.0};
  std::mutex status_mutex_;   // event_, camera_dev_

  std::thread capture_thread_, process_thread_, detect_thread_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr lane_pub_, det_pub_, state_pub_, arm_pub_;
  rclcpp::Publisher<sensor_msgs::msg::CompressedImage>::SharedPtr image_pub_;
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_pub_;
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr control_sub_, command_sub_;
  rclcpp::TimerBase::SharedPtr state_timer_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<VisionNode>();
  rclcpp::spin(node);
  node.reset();
  rclcpp::shutdown();
  return 0;
}
