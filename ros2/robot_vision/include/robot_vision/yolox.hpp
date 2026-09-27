// YOLOX object detection (COCO, 80 classes) with ncnn on the Pi's CPU.
// Models: YOLOX nano / tiny converted to ncnn (yoloxN.*, yoloxT.*), input 416x416.
#pragma once

#include <memory>
#include <string>
#include <vector>

#include <opencv2/core.hpp>

namespace ncnn {class Net;}

namespace robot_vision
{

struct Detection
{
  int label = 0;
  std::string name;
  float prob = 0.f;
  cv::Rect2f box;   // image pixels
};

extern const char * const kCocoClasses[80];

class Yolox
{
public:
  Yolox();
  ~Yolox();
  // e.g. load(dir + "/yoloxN.param", dir + "/yoloxN.bin", 3). False if a file is missing or bad.
  bool load(const std::string & param, const std::string & bin, int threads, int input_size = 416);
  bool loaded() const {return loaded_;}
  std::vector<Detection> detect(const cv::Mat & bgr, float prob_threshold = 0.4f,
    float nms_threshold = 0.45f) const;

private:
  std::unique_ptr<ncnn::Net> net_;
  int size_ = 416;
  bool loaded_ = false;
};

void draw_detections(cv::Mat & bgr, const std::vector<Detection> & dets);

}  // namespace robot_vision
