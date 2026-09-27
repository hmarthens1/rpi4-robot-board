// vision_bench: how fast are lane tracking and YOLOX on this Pi?
//
//   vision_bench [models_dir] [image] [runs]
//   defaults: /usr/local/share/robot_vision/models, <models_dir>/parking.jpg, 10
//
// Prints the milliseconds per frame for lane tracking (640x480) and for YOLOX
// nano and tiny with 1-4 threads, and what each model finds in the image.
#include <chrono>
#include <cstdio>
#include <string>

#include <opencv2/imgcodecs.hpp>
#include <opencv2/imgproc.hpp>

#include "robot_vision/lane.hpp"
#include "robot_vision/yolox.hpp"

using Clock = std::chrono::steady_clock;

template<typename F>
double time_ms(int runs, F f)
{
  f();   // warm-up
  const auto t0 = Clock::now();
  for (int i = 0; i < runs; ++i) {f();}
  return std::chrono::duration<double, std::milli>(Clock::now() - t0).count() / runs;
}

int main(int argc, char ** argv)
{
  const std::string dir = argc > 1 ? argv[1] : "/usr/local/share/robot_vision/models";
  const std::string image = argc > 2 ? argv[2] : dir + "/parking.jpg";
  const int runs = argc > 3 ? std::stoi(argv[3]) : 10;
  cv::Mat img = cv::imread(image);
  if (img.empty()) {
    std::fprintf(stderr, "can't read %s\n", image.c_str());
    return 1;
  }
  std::printf("image %s: %dx%d, %d runs each\n\n", image.c_str(), img.cols, img.rows, runs);

  cv::Mat frame;
  cv::resize(img, frame, {640, 480});
  robot_vision::LaneConfig cfg;
  robot_vision::color_preset("yellow", cfg.ranges);
  std::printf("lane tracking, 640x480 -> 320 px:  %6.1f ms\n\n",
    time_ms(runs * 10, [&] {robot_vision::detect_lane(frame, cfg);}));

  for (const std::string model : {"yoloxN", "yoloxT"}) {
    for (int threads = 1; threads <= 4; ++threads) {
      robot_vision::Yolox yolo;
      if (!yolo.load(dir + "/" + model + ".param", dir + "/" + model + ".bin", threads)) {
        std::printf("%s: can't load from %s\n", model.c_str(), dir.c_str());
        break;
      }
      std::vector<robot_vision::Detection> dets;
      const double ms = time_ms(runs, [&] {dets = yolo.detect(frame);});
      std::printf("%s  %d thread%s: %7.1f ms  (%4.1f frames/s)", model.c_str(), threads,
        threads > 1 ? "s" : " ", ms, 1000.0 / ms);
      if (threads == 4) {
        std::printf("  found:");
        for (const auto & d : dets) {std::printf(" %s %.0f%%", d.name.c_str(), d.prob * 100);}
      }
      std::printf("\n");
    }
  }
  return 0;
}
