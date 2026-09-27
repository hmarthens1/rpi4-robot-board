// Lane tracking on synthetic images, and YOLOX on the sample image when the models are installed.
#include <gtest/gtest.h>

#include <cmath>
#include <filesystem>

#include <opencv2/imgcodecs.hpp>
#include <opencv2/imgproc.hpp>

#include "robot_vision/lane.hpp"
#include "robot_vision/yolox.hpp"

using robot_vision::LaneConfig;

namespace
{
const cv::Scalar kYellowBgr(0, 220, 230), kFloor(90, 90, 90), kRedBgr(20, 20, 200);

LaneConfig config(const std::string & color = "yellow")
{
  LaneConfig c;
  EXPECT_TRUE(robot_vision::color_preset(color, c.ranges));
  return c;
}

cv::Mat floor_image() {return cv::Mat(480, 640, CV_8UC3, kFloor);}
}  // namespace

TEST(Lane, CentredLineHasNoOffset)
{
  auto img = floor_image();
  cv::rectangle(img, {305, 0}, {335, 479}, kYellowBgr, -1);
  const auto r = robot_vision::detect_lane(img, config());
  ASSERT_TRUE(r.found);
  EXPECT_EQ(r.bands_found, 4);
  EXPECT_NEAR(r.offset, 0.0, 0.03);
  EXPECT_NEAR(r.angle, 0.0, 0.03);
}

TEST(Lane, LineRightOfCentreGivesPositiveOffset)
{
  auto img = floor_image();
  cv::rectangle(img, {465, 0}, {495, 479}, kYellowBgr, -1);   // centre x = 480
  const auto r = robot_vision::detect_lane(img, config());
  ASSERT_TRUE(r.found);
  EXPECT_NEAR(r.offset, 0.5, 0.03);
}

TEST(Lane, LineHeadingRightGivesPositiveAngle)
{
  auto img = floor_image();
  cv::line(img, {320, 479}, {480, 240}, kYellowBgr, 30);   // leans right further up
  const auto r = robot_vision::detect_lane(img, config());
  ASSERT_TRUE(r.found);
  EXPECT_GT(r.angle, 0.3);
  const double expected = std::atan(160.0 / 239.0);
  EXPECT_NEAR(r.angle, expected, 0.1);
}

TEST(Lane, OtherColoursAndEmptyFloorAreNotALane)
{
  auto img = floor_image();
  EXPECT_FALSE(robot_vision::detect_lane(img, config()).found);
  cv::rectangle(img, {305, 0}, {335, 479}, cv::Scalar(200, 60, 20), -1);   // blue tape
  EXPECT_FALSE(robot_vision::detect_lane(img, config("yellow")).found);
  EXPECT_TRUE(robot_vision::detect_lane(img, config("blue")).found);
}

TEST(Lane, RedWrapsAroundHue)
{
  // OpenCV hue runs 0..180, and red sits at both ends
  for (const auto & bgr : {kRedBgr, cv::Scalar(40, 20, 200)}) {   // hue 0 and about 177
    auto img = floor_image();
    cv::rectangle(img, {305, 0}, {335, 479}, bgr, -1);
    EXPECT_TRUE(robot_vision::detect_lane(img, config("red")).found);
  }
}

TEST(Lane, LineAboveTheSearchAreaIsIgnored)
{
  auto img = floor_image();
  cv::rectangle(img, {305, 0}, {335, 200}, kYellowBgr, -1);   // top part only
  EXPECT_FALSE(robot_vision::detect_lane(img, config()).found);
}

TEST(Lane, TwoLinesGiveTheMiddle)
{
  auto img = floor_image();
  cv::rectangle(img, {100, 0}, {130, 479}, kYellowBgr, -1);
  cv::rectangle(img, {590, 0}, {620, 479}, kYellowBgr, -1);   // middle x = 360
  auto cfg = config();
  cfg.two_lines = true;
  const auto r = robot_vision::detect_lane(img, cfg);
  ASSERT_TRUE(r.found);
  EXPECT_NEAR(r.offset, 40.0 / 320.0, 0.03);
}

TEST(Lane, UnknownPresetIsRejected)
{
  std::vector<robot_vision::HsvRange> r;
  EXPECT_FALSE(robot_vision::color_preset("purple", r));
}

TEST(Yolox, FindsCarsInTheSampleImage)
{
  const std::string dir = "/usr/local/share/robot_vision/models";
  if (!std::filesystem::exists(dir + "/yoloxN.bin")) {GTEST_SKIP() << "models not installed";}
  robot_vision::Yolox yolo;
  ASSERT_TRUE(yolo.load(dir + "/yoloxN.param", dir + "/yoloxN.bin", 4));
  const auto img = cv::imread(dir + "/parking.jpg");
  ASSERT_FALSE(img.empty());
  const auto dets = yolo.detect(img, 0.4f);
  int cars = 0;
  for (const auto & d : dets) {
    cars += d.name == "car";
    EXPECT_GE(d.box.x, 0);
    EXPECT_LE(d.box.x + d.box.width, img.cols);
  }
  EXPECT_GE(cars, 1);
}
