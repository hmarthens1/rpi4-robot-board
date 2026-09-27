#include "robot_vision/lane.hpp"

#include <algorithm>
#include <cmath>
#include <map>

#include <opencv2/imgproc.hpp>

namespace robot_vision
{

namespace
{
const std::map<std::string, std::vector<HsvRange>> & presets()
{
  static const std::map<std::string, std::vector<HsvRange>> p = {
    {"yellow", {{{18, 80, 80}, {35, 255, 255}}}},
    {"green", {{{40, 60, 50}, {85, 255, 255}}}},
    {"blue", {{{95, 100, 50}, {130, 255, 255}}}},
    {"red", {{{0, 100, 70}, {10, 255, 255}}, {{170, 100, 70}, {180, 255, 255}}}},
    {"black", {{{0, 0, 0}, {180, 255, 70}}}},
    {"white", {{{0, 0, 170}, {180, 50, 255}}}},
  };
  return p;
}
}  // namespace

bool color_preset(const std::string & name, std::vector<HsvRange> & ranges)
{
  const auto it = presets().find(name);
  if (it == presets().end()) {return false;}
  ranges = it->second;
  return true;
}

std::vector<std::string> color_presets()
{
  std::vector<std::string> names;
  for (const auto & [name, _] : presets()) {names.push_back(name);}
  return names;
}

LaneResult detect_lane(const cv::Mat & bgr, const LaneConfig & cfg)
{
  LaneResult r;
  if (bgr.empty() || cfg.ranges.empty()) {return r;}
  const double scale = std::min(1.0, static_cast<double>(cfg.width) / bgr.cols);
  cv::Mat small;
  if (scale < 1.0) {
    cv::resize(bgr, small, cv::Size(), scale, scale, cv::INTER_AREA);
  } else {
    small = bgr;
  }
  const int y0 = std::clamp(static_cast<int>(cfg.roi_top * small.rows), 0, small.rows - 1);
  cv::Mat roi = small.rowRange(y0, small.rows), hsv;
  cv::GaussianBlur(roi, hsv, cv::Size(5, 5), 0);
  cv::cvtColor(hsv, hsv, cv::COLOR_BGR2HSV);

  cv::Mat mask = cv::Mat::zeros(hsv.size(), CV_8U), part;
  for (const auto & range : cfg.ranges) {
    cv::inRange(hsv, range.lo, range.hi, part);
    mask |= part;
  }
  cv::morphologyEx(mask, mask, cv::MORPH_OPEN, cv::getStructuringElement(cv::MORPH_RECT, {3, 3}));
  r.mask = mask;
  r.coverage = static_cast<double>(cv::countNonZero(mask)) / std::max<size_t>(1, mask.total());

  const int bands = std::max(1, cfg.bands);
  const int band_h = mask.rows / bands;
  if (band_h < 2) {return r;}
  std::vector<double> weights;
  for (int b = 0; b < bands; ++b) {
    cv::Mat band = mask.rowRange(b * band_h, (b + 1) * band_h).clone();
    std::vector<std::vector<cv::Point>> contours;
    cv::findContours(band, contours, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE);
    const double min_px = cfg.min_area * band.total();
    std::vector<std::pair<double, double>> blobs;   // (x centre, area)
    for (const auto & c : contours) {
      const auto m = cv::moments(c);
      if (m.m00 >= min_px) {blobs.emplace_back(m.m10 / m.m00, m.m00);}
    }
    if (blobs.empty()) {continue;}
    double x;
    if (cfg.two_lines) {
      if (blobs.size() < 2) {continue;}   // one line alone doesn't say where the lane's middle is
      const auto [lo, hi] = std::minmax_element(blobs.begin(), blobs.end());
      x = (lo->first + hi->first) / 2.0;
    } else {
      x = std::max_element(blobs.begin(), blobs.end(),
          [](const auto & a, const auto & b) {return a.second < b.second;})->first;
    }
    const double y = y0 + b * band_h + band_h / 2.0;
    r.points.emplace_back(static_cast<float>(x / scale), static_cast<float>(y / scale));
    weights.push_back(b + 1.0);   // the bands nearest the robot count most
  }
  r.bands_found = static_cast<int>(r.points.size());
  if (r.points.empty()) {return r;}
  r.found = true;

  double sx = 0, sw = 0;
  for (size_t i = 0; i < r.points.size(); ++i) {
    sx += weights[i] * r.points[i].x;
    sw += weights[i];
  }
  const double half = bgr.cols / 2.0;
  r.offset = std::clamp((sx / sw - half) / half, -1.0, 1.0);

  if (r.points.size() >= 2) {
    // Least squares x = a*y + c; going up the image (y falling) x changes by -a.
    double my = 0, mx = 0;
    for (const auto & p : r.points) {my += p.y; mx += p.x;}
    my /= r.points.size();
    mx /= r.points.size();
    double num = 0, den = 0;
    for (const auto & p : r.points) {
      num += (p.y - my) * (p.x - mx);
      den += (p.y - my) * (p.y - my);
    }
    if (den > 0) {r.angle = std::atan(-num / den);}
  }
  return r;
}

void draw_lane(cv::Mat & bgr, const LaneResult & lane, const LaneConfig & cfg)
{
  const int y0 = static_cast<int>(cfg.roi_top * bgr.rows);
  cv::rectangle(bgr, {0, y0}, {bgr.cols - 1, bgr.rows - 1}, {255, 200, 0}, 1);
  cv::line(bgr, {bgr.cols / 2, y0}, {bgr.cols / 2, bgr.rows - 1}, {200, 200, 200}, 1);
  for (const auto & p : lane.points) {cv::circle(bgr, p, 6, {0, 0, 255}, -1);}
  for (size_t i = 1; i < lane.points.size(); ++i) {
    cv::line(bgr, lane.points[i - 1], lane.points[i], {0, 0, 255}, 2);
  }
  char text[64];
  if (lane.found) {
    std::snprintf(text, sizeof text, "offset %+.2f  angle %+.0f deg", lane.offset, lane.angle * 180 / M_PI);
  } else {
    std::snprintf(text, sizeof text, "no lane");
  }
  cv::putText(bgr, text, {8, y0 - 8}, cv::FONT_HERSHEY_SIMPLEX, 0.6, {0, 0, 0}, 3);
  cv::putText(bgr, text, {8, y0 - 8}, cv::FONT_HERSHEY_SIMPLEX, 0.6, {255, 255, 255}, 1);
}

}  // namespace robot_vision
