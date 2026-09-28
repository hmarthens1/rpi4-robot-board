// LaneSeeker: search with the pan servo, turn to face the lane, follow, search again.
#include <gtest/gtest.h>

#include <cmath>
#include <functional>

#include "robot_vision/lane_seeker.hpp"

using robot_vision::LaneResult;
using robot_vision::LaneSeeker;
using robot_vision::SeekPhase;
using robot_vision::SeekStep;

namespace
{
LaneResult lane_at(double offset, double angle = 0.0)
{
  LaneResult r;
  r.found = true;
  r.offset = offset;
  r.angle = angle;
  r.bands_found = 4;
  return r;
}

LaneResult none() {return LaneResult();}

// A little world: the lane lies at `lane_deg` from the robot's heading (+ = left).
// The camera sees it when it is within the field of view of the pan direction.
struct World
{
  explicit World(double lane) : lane_deg(lane) {}
  double lane_deg;
  double heading = 0.0;   // robot, degrees, + = left
  int base = 1500;
  robot_vision::SeekConfig cfg;

  LaneResult view() const
  {
    const double pan = (base - cfg.base_centre) / cfg.base_us_per_deg;
    const double rel = lane_deg - heading - pan;        // + = left of the camera axis
    const double offset = -std::tan(rel * M_PI / 180) / cfg.tan_half_fov;
    if (std::abs(rel) > 28 || std::abs(offset) > 1) {return none();}
    return lane_at(offset);
  }

  void apply(const SeekStep & s, double dt)
  {
    if (s.base) {base = *s.base;}
    if (s.drive) {heading += s.turn * cfg.turn_deg_per_s * dt;}
  }
};

// Runs the seeker in the world at 20 frames/s until `done` or `seconds` pass.
double run(LaneSeeker & seeker, World & w, double t, double seconds,
  const std::function<bool()> & done)
{
  const double dt = 0.05;
  for (double end = t + seconds; t < end && !done(); t += dt) {
    w.apply(seeker.update(t, w.view()), dt);
  }
  return t;
}
}  // namespace

TEST(Steer, NeverBelowTheSpeedTheWheelsNeed)
{
  robot_vision::SeekConfig cfg;
  double forward, turn;
  robot_vision::steer(lane_at(0.0), cfg, forward, turn);
  EXPECT_NEAR(turn, 0.0, 1e-9);
  EXPECT_NEAR(forward, cfg.speed, 1e-9);
  robot_vision::steer(lane_at(0.5), cfg, forward, turn);   // line to the right: turn right
  EXPECT_LT(turn, 0.0);
  EXPECT_GE(forward, cfg.min_drive);
  cfg.speed = 0.25;                                          // what failed on robot01
  robot_vision::steer(lane_at(0.2), cfg, forward, turn);
  EXPECT_GE(forward, cfg.min_drive);
}

TEST(Steer, SharpCurvePivotsInPlace)
{
  robot_vision::SeekConfig cfg;
  double forward, turn;
  robot_vision::steer(lane_at(-1.0, -0.5), cfg, forward, turn);   // far left, heading left
  EXPECT_EQ(forward, 0.0);
  EXPECT_GE(turn, cfg.min_drive);
}

TEST(Seeker, LaneAheadFollowsAfterOneLook)
{
  LaneSeeker s;
  const auto first = s.start(0.0, true);
  ASSERT_TRUE(first.base);
  EXPECT_EQ(*first.base, 1500);                // centre first
  EXPECT_EQ(s.phase(), SeekPhase::Sweep);
  EXPECT_TRUE(s.update(0.1, lane_at(0.0)).drive);   // still settling: stands still
  EXPECT_EQ(s.phase(), SeekPhase::Sweep);
  double t = 0.8;
  for (int i = 0; i < 3; ++i, t += 0.05) {s.update(t, lane_at(0.05));}
  EXPECT_EQ(s.phase(), SeekPhase::Follow);
  const auto step = s.update(t, lane_at(0.0));
  EXPECT_TRUE(step.drive);
  EXPECT_GE(step.forward, s.config().min_drive);
}

TEST(Seeker, SweepOrderPrefersOneSide)
{
  LaneSeeker s;
  std::vector<int> seen;
  double t = 0;
  seen.push_back(*s.start(t, true).base);
  while (s.phase() == SeekPhase::Sweep && t < 20) {
    t += 0.05;
    const auto step = s.update(t, none());
    if (step.base && s.phase() == SeekPhase::Sweep) {seen.push_back(*step.base);}
  }
  const std::vector<int> expected{1500, 1700, 1300, 1900, 1100, 2100, 900};   // left first
  EXPECT_EQ(seen, expected);
  EXPECT_EQ(s.phase(), SeekPhase::Turn);       // nothing: turn the robot and look again
}

TEST(Seeker, FindsALaneToTheSideTurnsAndFollows)
{
  for (double lane_deg : {35.0, -40.0, 60.0, -75.0}) {
    SCOPED_TRACE(lane_deg);
    LaneSeeker s;
    World w{lane_deg};
    w.apply(s.start(0.0, true), 0.0);
    run(s, w, 0.0, 30.0, [&] {return s.phase() == SeekPhase::Follow || s.phase() == SeekPhase::Failed;});
    ASSERT_EQ(s.phase(), SeekPhase::Follow);
    EXPECT_EQ(w.base, 1500);                   // camera centred for following
    EXPECT_NEAR(w.heading, lane_deg, s.config().align_deg);
  }
}

TEST(Seeker, LaneBehindTheRobotIsFoundAfterSpinning)
{
  LaneSeeker s;
  World w{180.0};
  w.apply(s.start(0.0, true), 0.0);
  run(s, w, 0.0, 60.0, [&] {return s.phase() == SeekPhase::Follow || s.phase() == SeekPhase::Failed;});
  ASSERT_EQ(s.phase(), SeekPhase::Follow);
  EXPECT_NEAR(std::remainder(w.heading - 180.0, 360.0), 0.0, s.config().align_deg);
}

TEST(Seeker, GivesUpAfterAFullTurnWithoutALane)
{
  LaneSeeker s;
  double t = 0;
  s.start(t, true);
  for (; t < 120 && s.phase() != SeekPhase::Failed; t += 0.05) {s.update(t, none());}
  EXPECT_EQ(s.phase(), SeekPhase::Failed);
  EXPECT_FALSE(s.active());
  EXPECT_FALSE(s.update(t, lane_at(0.0)).drive);   // stays put
}

TEST(Seeker, WithoutSearchALostLaneStops)
{
  LaneSeeker s;
  s.start(0.0, false);
  EXPECT_EQ(s.phase(), SeekPhase::Follow);
  s.update(0.1, lane_at(0.6));
  EXPECT_EQ(s.update(0.5, none()).forward, 0.0);   // briefly gone: stand still
  s.update(1.0, none());
  EXPECT_EQ(s.phase(), SeekPhase::Follow);
  const auto step = s.update(1.2, none());         // > lost_timeout
  EXPECT_EQ(s.phase(), SeekPhase::Failed);
  EXPECT_EQ(step.event, "lane lost");
}

TEST(Seeker, LostWithSearchSweepsRightFirst)
{
  LaneSeeker s;
  double t = 0;
  s.start(t, true);
  for (int i = 0; i < 40 && s.phase() != SeekPhase::Follow; ++i) {s.update(t += 0.05, lane_at(0.0));}
  ASSERT_EQ(s.phase(), SeekPhase::Follow);
  s.update(t += 0.05, lane_at(0.6));           // last seen right of centre
  SeekStep step;
  while (s.phase() == SeekPhase::Follow && t < 10) {step = s.update(t += 0.05, none());}
  EXPECT_EQ(s.phase(), SeekPhase::Sweep);
  ASSERT_TRUE(step.base);
  EXPECT_EQ(*step.base, 1500);
  step = SeekStep();
  while (!step.base && t < 20) {step = s.update(t += 0.05, none());}
  ASSERT_TRUE(step.base);
  EXPECT_EQ(*step.base, 1300);                 // right (lower pulse) before left
}

TEST(Seeker, StopEndsWithAZeroTwist)
{
  LaneSeeker s;
  s.start(0.0, true);
  const auto step = s.stop("stop command");
  EXPECT_TRUE(step.drive);
  EXPECT_EQ(step.forward, 0.0);
  EXPECT_EQ(step.turn, 0.0);
  EXPECT_EQ(s.phase(), SeekPhase::Idle);
  EXPECT_FALSE(s.stop("again").drive);         // nothing to stop
}
