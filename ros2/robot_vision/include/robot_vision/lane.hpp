// Colour lane tracking with OpenCV: find a coloured line (tape) in the bottom of
// the camera image and say how far it is from the centre and which way it heads.
//
//   image -> shrink to `width` px -> bottom part (from roi_top) -> blur -> colour
//   mask (see below) -> split into horizontal bands -> in each band the
//   biggest blob's centre (or, with two_lines, the middle between the leftmost
//   and rightmost blob) -> offset and angle from those points.
//
// Colour mask: the named colours (yellow, green, blue, red) are matched in Lab
// *relative to the floor*: a pixel matches when its colour differs from the
// search area's median colour by at least `min_chroma` in the colour's direction
// (hue angle in the a*b* plane). Fixed HSV thresholds failed on robot01: the
// camera over-exposes pale tape on a grey floor (V 250, S 30-65, below the
// preset's S >= 80), but it is still clearly "more yellow than the floor".
// black, white and custom `hsv` ranges use plain HSV thresholds.
#pragma once

#include <optional>

#include <string>
#include <vector>

#include <opencv2/core.hpp>

namespace robot_vision
{

struct HsvRange
{
  cv::Scalar lo, hi;   // OpenCV HSV: H 0..180, S and V 0..255
};

// A colour as a direction in Lab's a*b* plane (OpenCV 8-bit Lab, a and b - 128).
struct LabHue
{
  double angle_deg;         // atan2(b, a): red ~35, yellow ~105, green ~147, blue ~-70
  double tol_deg;           // accepted spread around it
  double min_chroma;        // how much more of this colour than the floor, in Lab units
};

struct LaneConfig
{
  std::optional<LabHue> lab;      // set: match relative to the floor in Lab; unset: `ranges` (HSV)
  std::vector<HsvRange> ranges;   // a pixel matches if it is in any range (red wraps around H=180)
  double roi_top = 0.5;           // the search area starts this far down the image (0..1)
  int bands = 4;                  // horizontal bands in the search area
  double min_area = 0.01;         // smallest blob, as a fraction of a band's area
  bool two_lines = false;         // lane between two lines of the colour, instead of one line
  int width = 320;                // processing width in pixels (the image is shrunk to this)
};

struct LaneResult
{
  bool found = false;
  double offset = 0.0;            // -1 (far left) .. 1 (far right) of the image centre
  double angle = 0.0;             // radians; > 0: the line heads to the right further ahead
  int bands_found = 0;
  std::vector<cv::Point2f> points;   // one per band with a match, top to bottom, image pixels
  double coverage = 0.0;          // fraction of the search area that matched the colour
  cv::Mat mask;                   // the colour mask of the search area (processing size)
};

// Named colours: yellow, green, blue, red, black, white. False if unknown.
// The LaneConfig overload also sets `lab` (and clears it for black/white).
bool color_preset(const std::string & name, std::vector<HsvRange> & ranges);
bool color_preset(const std::string & name, LaneConfig & cfg);
std::vector<std::string> color_presets();

LaneResult detect_lane(const cv::Mat & bgr, const LaneConfig & cfg);

// Draws the search area, the matched points and the offset onto `bgr`.
void draw_lane(cv::Mat & bgr, const LaneResult & lane, const LaneConfig & cfg);

}  // namespace robot_vision
