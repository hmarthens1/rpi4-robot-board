// YOLOX decoding follows ncnn's examples/yolox.cpp (BSD-3-Clause, Tencent) and
// YOLOX's yolo_head.py (Apache-2.0, Megvii): input is BGR 0..255, letterboxed to
// a square padded with 114 on the right and bottom; output rows are
// [cx, cy, w, h, objectness, 80 class scores] over strides 8, 16 and 32.
#include "robot_vision/yolox.hpp"

#include <algorithm>
#include <cmath>
#include <cstdio>

#include <opencv2/imgproc.hpp>

#include "layer.h"
#include "net.h"

namespace robot_vision
{

const char * const kCocoClasses[80] = {
  "person", "bicycle", "car", "motorcycle", "airplane", "bus", "train", "truck", "boat",
  "traffic light", "fire hydrant", "stop sign", "parking meter", "bench", "bird", "cat", "dog",
  "horse", "sheep", "cow", "elephant", "bear", "zebra", "giraffe", "backpack", "umbrella",
  "handbag", "tie", "suitcase", "frisbee", "skis", "snowboard", "sports ball", "kite",
  "baseball bat", "baseball glove", "skateboard", "surfboard", "tennis racket", "bottle",
  "wine glass", "cup", "fork", "knife", "spoon", "bowl", "banana", "apple", "sandwich", "orange",
  "broccoli", "carrot", "hot dog", "pizza", "donut", "cake", "chair", "couch", "potted plant",
  "bed", "dining table", "toilet", "tv", "laptop", "mouse", "remote", "keyboard", "cell phone",
  "microwave", "oven", "toaster", "sink", "refrigerator", "book", "clock", "vase", "scissors",
  "teddy bear", "hair drier", "toothbrush"};

namespace
{
// The models' first layer: space-to-depth, as in YOLOv5 (not built into ncnn).
class YoloV5Focus : public ncnn::Layer
{
public:
  YoloV5Focus() {one_blob_only = true;}

  int forward(const ncnn::Mat & bottom, ncnn::Mat & top, const ncnn::Option & opt) const override
  {
    const int w = bottom.w, channels = bottom.c;
    const int outw = w / 2, outh = bottom.h / 2, outc = channels * 4;
    top.create(outw, outh, outc, 4u, 1, opt.blob_allocator);
    if (top.empty()) {return -100;}
    #pragma omp parallel for num_threads(opt.num_threads)
    for (int p = 0; p < outc; p++) {
      const float * ptr = bottom.channel(p % channels).row((p / channels) % 2) + ((p / channels) / 2);
      float * out = top.channel(p);
      for (int i = 0; i < outh; i++) {
        for (int j = 0; j < outw; j++) {
          *out++ = *ptr;
          ptr += 2;
        }
        ptr += w;
      }
    }
    return 0;
  }
};

DEFINE_LAYER_CREATOR(YoloV5Focus)

float iou(const cv::Rect2f & a, const cv::Rect2f & b)
{
  const float inter = (a & b).area();
  return inter / (a.area() + b.area() - inter);
}
}  // namespace

Yolox::Yolox() = default;
Yolox::~Yolox() = default;

bool Yolox::load(const std::string & param, const std::string & bin, int threads, int input_size)
{
  loaded_ = false;
  net_ = std::make_unique<ncnn::Net>();
  net_->opt.use_vulkan_compute = false;
  net_->opt.lightmode = true;
  net_->opt.num_threads = std::max(1, threads);
  net_->register_custom_layer("YoloV5Focus", YoloV5Focus_layer_creator);
  if (net_->load_param(param.c_str()) != 0 || net_->load_model(bin.c_str()) != 0) {return false;}
  size_ = input_size;
  loaded_ = true;
  return true;
}

std::vector<Detection> Yolox::detect(const cv::Mat & bgr, float prob_threshold, float nms_threshold) const
{
  std::vector<Detection> out;
  if (!loaded_ || bgr.empty()) {return out;}
  const int img_w = bgr.cols, img_h = bgr.rows;
  const float scale = std::min(static_cast<float>(size_) / img_w, static_cast<float>(size_) / img_h);
  const int w = static_cast<int>(img_w * scale), h = static_cast<int>(img_h * scale);

  ncnn::Mat in = ncnn::Mat::from_pixels_resize(bgr.data, ncnn::Mat::PIXEL_BGR, img_w, img_h, w, h);
  ncnn::Mat in_pad;
  ncnn::copy_make_border(in, in_pad, 0, size_ - h, 0, size_ - w, ncnn::BORDER_CONSTANT, 114.f);

  ncnn::Extractor ex = net_->create_extractor();
  ex.input("images", in_pad);
  ncnn::Mat feat;
  if (ex.extract("output", feat) != 0) {return out;}

  // Proposals over the grid of each stride, in the same order as the model's output rows.
  std::vector<Detection> props;
  const int num_class = feat.w - 5;
  const float * row = feat.channel(0);
  for (int stride : {8, 16, 32}) {
    const int n = size_ / stride;
    for (int gy = 0; gy < n; gy++) {
      for (int gx = 0; gx < n; gx++, row += feat.w) {
        const float obj = row[4];
        if (obj < prob_threshold) {continue;}   // class score <= 1, so it can't pass
        const int best = static_cast<int>(std::max_element(row + 5, row + 5 + num_class) - (row + 5));
        const float prob = obj * row[5 + best];
        if (prob < prob_threshold) {continue;}
        const float cx = (row[0] + gx) * stride, cy = (row[1] + gy) * stride;
        const float bw = std::exp(row[2]) * stride, bh = std::exp(row[3]) * stride;
        Detection d;
        d.label = best;
        d.prob = prob;
        d.box = {cx - bw / 2, cy - bh / 2, bw, bh};
        props.push_back(d);
      }
    }
  }
  std::sort(props.begin(), props.end(), [](const auto & a, const auto & b) {return a.prob > b.prob;});
  for (const auto & p : props) {
    bool keep = true;
    for (const auto & k : out) {
      if (k.label == p.label && iou(k.box, p.box) > nms_threshold) {keep = false; break;}
    }
    if (keep) {out.push_back(p);}
  }
  for (auto & d : out) {
    // back to the original image, clipped to it
    float x0 = d.box.x / scale, y0 = d.box.y / scale;
    float x1 = (d.box.x + d.box.width) / scale, y1 = (d.box.y + d.box.height) / scale;
    x0 = std::clamp(x0, 0.f, img_w - 1.f);
    y0 = std::clamp(y0, 0.f, img_h - 1.f);
    x1 = std::clamp(x1, 0.f, img_w - 1.f);
    y1 = std::clamp(y1, 0.f, img_h - 1.f);
    d.box = {x0, y0, x1 - x0, y1 - y0};
    d.name = (d.label >= 0 && d.label < 80) ? kCocoClasses[d.label] : "?";
  }
  return out;
}

void draw_detections(cv::Mat & bgr, const std::vector<Detection> & dets)
{
  for (const auto & d : dets) {
    cv::rectangle(bgr, d.box, {0, 255, 0}, 2);
    char text[64];
    std::snprintf(text, sizeof text, "%s %.0f%%", d.name.c_str(), d.prob * 100);
    const cv::Point org(static_cast<int>(d.box.x), std::max(12, static_cast<int>(d.box.y) - 4));
    cv::putText(bgr, text, org, cv::FONT_HERSHEY_SIMPLEX, 0.5, {0, 0, 0}, 3);
    cv::putText(bgr, text, org, cv::FONT_HERSHEY_SIMPLEX, 0.5, {0, 255, 0}, 1);
  }
}

}  // namespace robot_vision
