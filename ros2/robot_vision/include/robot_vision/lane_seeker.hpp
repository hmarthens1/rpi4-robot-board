// Find a lane with a camera on a panning servo (robot01's arm base), face it
// with the wheels, then follow it; search again when it is lost.
//
//   Sweep   the camera pans through `pan_steps` (centre first, then out to
//           both sides); at each step it waits `settle_s` and then judges
//           `looks` frames. A lane seen in `need` of them gives a bearing:
//             bearing = pan angle - atan(offset * tan_half_fov)   (+ = left)
//   Turn    the robot turns in place towards that bearing (open loop, at
//           `turn_deg_per_s`) while the camera pans back to the centre, then
//           Sweep again from the centre to confirm. If the lane is ahead
//           (|bearing| < align_deg with the camera centred) -> Follow.
//           A sweep that finds nothing turns the robot by `spin_deg` and
//           sweeps again, up to a full turn.
//   Follow  steer on the lane with the camera centred; when it is lost for
//           `lost_timeout` s, Sweep again, the side it was last seen first.
//   Failed  nothing found after all attempts: the robot stands still.
//
// No ROS here: the node feeds it one lane result per frame and does what the
// returned Step says (pan the servo, publish the twist).
#pragma once

#include <optional>
#include <string>
#include <vector>

#include "robot_vision/lane.hpp"

namespace robot_vision
{

struct SeekConfig
{
  // Camera pan servo (robot01: base servo 6; higher pulse = left). The steps are
  // relative to base_centre, the pulse that looks straight ahead.
  std::vector<int> pan_steps{0, 200, -200, 400, -400, 600, -600};
  int base_centre = 1500;
  double base_us_per_deg = 11.71;   // measured on robot01 (fleet_vision.triangulate)
  double tan_half_fov = 0.584;      // 160 px / f = 274 px at 320 px wide
  double settle_s = 0.7;            // after a pan before judging frames
  int looks = 4;                    // frames judged per pan step ...
  int need = 3;                     // ... of which the lane must be in this many
  int min_bands = 2;                // a lane needs points in this many bands
  // Turning with the wheels (fractions of command_node's max_speed).
  double turn_deg_per_s = 150.0;    // robot01 at full command on carpet
  double align_deg = 12.0;          // close enough to start following
  double spin_deg = 90.0;           // turn after an empty sweep
  int max_spins = 4;                // empty sweeps before giving up (4 x 90 = a full turn)
  int max_turns = 5;                // turns towards a seen lane before giving up
  double lost_timeout = 1.0;        // s without the lane while following
  // Following.
  double speed = 0.7;               // forward, fraction of max_speed
  double min_drive = 0.6;           // robot01 doesn't move below this
  double kp = 0.8, ka = 0.5;        // turn = -(kp * offset + ka * angle)
  double pivot_turn = 0.7;          // above this, turn in place instead of driving on
};

enum class SeekPhase {Idle, Sweep, Turn, Follow, Failed};
const char * to_string(SeekPhase p);

struct SeekStep
{
  std::optional<int> base;          // pan the camera to this pulse
  bool drive = false;               // publish (forward, turn); false: leave cmd_vel alone
  double forward = 0.0, turn = 0.0; // fractions of max_speed; turn > 0 = left (counter-clockwise)
  std::string event;                // something to log, or empty
};

// Forward and turn commands for a lane seen by a centred camera.
void steer(const LaneResult & lane, const SeekConfig & cfg, double & forward, double & turn);

class LaneSeeker
{
public:
  explicit LaneSeeker(SeekConfig cfg = {}) : cfg_(std::move(cfg)) {}

  // search = false: follow straight away (the camera must already see the lane).
  SeekStep start(double now, bool search);
  SeekStep stop(const std::string & why);
  SeekStep update(double now, const LaneResult & lane);

  SeekPhase phase() const {return phase_;}
  bool active() const {return phase_ == SeekPhase::Sweep || phase_ == SeekPhase::Turn || phase_ == SeekPhase::Follow;}
  SeekConfig & config() {return cfg_;}
  const SeekConfig & config() const {return cfg_;}

private:
  SeekStep begin_sweep(double now, int prefer_side);
  SeekStep begin_turn(double now, double degrees);
  SeekStep judge(double now);
  bool good(const LaneResult & lane) const;

  SeekConfig cfg_;
  SeekPhase phase_ = SeekPhase::Idle;
  bool search_ = true;
  std::vector<int> order_;          // pan steps of the current sweep
  size_t step_ = 0;
  double settle_until_ = 0, turn_until_ = 0, turn_dir_ = 0, last_seen_ = 0;
  int looked_ = 0, hits_ = 0, spins_ = 0, turns_ = 0;
  double offset_sum_ = 0;
  int last_side_ = 1;               // +1: lane last seen to the left, -1: right
};

}  // namespace robot_vision
