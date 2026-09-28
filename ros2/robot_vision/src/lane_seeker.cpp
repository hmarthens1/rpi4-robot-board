#include "robot_vision/lane_seeker.hpp"

#include <algorithm>
#include <cmath>
#include <cstdio>

namespace robot_vision
{

const char * to_string(SeekPhase p)
{
  switch (p) {
    case SeekPhase::Idle: return "idle";
    case SeekPhase::Sweep: return "search";
    case SeekPhase::Turn: return "turn";
    case SeekPhase::Follow: return "follow";
    case SeekPhase::Failed: return "failed";
  }
  return "?";
}

void steer(const LaneResult & lane, const SeekConfig & cfg, double & forward, double & turn)
{
  // Line to the right (offset > 0) or heading right (angle > 0): turn right (negative).
  turn = std::clamp(-(cfg.kp * lane.offset + cfg.ka * lane.angle), -1.0, 1.0);
  if (std::abs(turn) > cfg.pivot_turn) {
    // Sharp: turn in place. A slowed-down forward speed would fall below
    // min_drive and the wheels would stall.
    forward = 0.0;
    turn = std::copysign(std::max(std::abs(turn), cfg.min_drive), turn);
  } else {
    forward = std::max(cfg.min_drive, cfg.speed * (1.0 - 0.5 * std::abs(turn)));
  }
}

bool LaneSeeker::good(const LaneResult & lane) const
{
  return lane.found && lane.bands_found >= cfg_.min_bands;
}

SeekStep LaneSeeker::start(double now, bool search)
{
  search_ = search;
  spins_ = turns_ = 0;
  last_side_ = 1;
  if (!search) {
    phase_ = SeekPhase::Follow;
    last_seen_ = now;
    SeekStep s;
    s.event = "following";
    return s;
  }
  auto s = begin_sweep(now, last_side_);
  s.event = "searching for the lane";
  return s;
}

SeekStep LaneSeeker::stop(const std::string & why)
{
  SeekStep s;
  if (active()) {
    s.drive = true;              // one last zero twist
    s.event = "stopped: " + why;
  }
  phase_ = SeekPhase::Idle;
  return s;
}

SeekStep LaneSeeker::begin_sweep(double now, int prefer_side)
{
  // Centre first, then each distance from the centre with the preferred side first.
  order_ = cfg_.base_steps;
  const int c = cfg_.base_centre;
  std::stable_sort(order_.begin(), order_.end(), [&](int a, int b) {
      const int da = std::abs(a - c), db = std::abs(b - c);
      if (da != db) {return da < db;}
      return (a - c) * prefer_side > (b - c) * prefer_side;   // higher pulse = left = side +1
    });
  phase_ = SeekPhase::Sweep;
  step_ = 0;
  looked_ = hits_ = 0;
  offset_sum_ = 0;
  settle_until_ = now + cfg_.settle_s;
  SeekStep s;
  s.base = order_.empty() ? c : order_[0];
  s.drive = true;                // stand still while looking
  return s;
}

SeekStep LaneSeeker::begin_turn(double now, double degrees)
{
  phase_ = SeekPhase::Turn;
  turn_dir_ = degrees >= 0 ? 1.0 : -1.0;
  turn_until_ = now + std::abs(degrees) / cfg_.turn_deg_per_s;
  SeekStep s;
  s.base = cfg_.base_centre;     // pan back while the body turns: the view stays on the lane
  s.drive = true;
  s.turn = turn_dir_;
  return s;
}

SeekStep LaneSeeker::judge(double now)
{
  const int base = order_[step_];
  if (hits_ >= cfg_.need) {
    const double offset = offset_sum_ / hits_;
    const double bearing = (base - cfg_.base_centre) / cfg_.base_us_per_deg -
      std::atan(offset * cfg_.tan_half_fov) * 180.0 / M_PI;
    last_side_ = bearing >= 0 ? 1 : -1;
    char buf[96];
    if (base == cfg_.base_centre && std::abs(bearing) < cfg_.align_deg) {
      phase_ = SeekPhase::Follow;
      last_seen_ = now;
      turns_ = 0;
      std::snprintf(buf, sizeof buf, "lane ahead (%+.0f deg): following", bearing);
      SeekStep s;
      s.event = buf;
      return s;
    }
    if (++turns_ > cfg_.max_turns) {
      phase_ = SeekPhase::Failed;
      SeekStep s;
      s.drive = true;
      s.event = "lane seen but could not face it";
      return s;
    }
    std::snprintf(buf, sizeof buf, "lane at %+.0f deg (pan %d): turning", bearing, base);
    auto s = begin_turn(now, bearing);
    s.event = buf;
    return s;
  }
  if (++step_ < order_.size()) {
    looked_ = hits_ = 0;
    offset_sum_ = 0;
    settle_until_ = now + cfg_.settle_s;
    SeekStep s;
    s.base = order_[step_];
    s.drive = true;
    return s;
  }
  if (++spins_ > cfg_.max_spins) {
    phase_ = SeekPhase::Failed;
    SeekStep s;
    s.base = cfg_.base_centre;
    s.drive = true;
    s.event = "no lane found all around";
    return s;
  }
  char buf[64];
  std::snprintf(buf, sizeof buf, "no lane in view: turning %+.0f deg", last_side_ * cfg_.spin_deg);
  auto s = begin_turn(now, last_side_ * cfg_.spin_deg);
  s.event = buf;
  return s;
}

SeekStep LaneSeeker::update(double now, const LaneResult & lane)
{
  SeekStep s;
  switch (phase_) {
    case SeekPhase::Idle:
    case SeekPhase::Failed:
      return s;

    case SeekPhase::Sweep:
      s.drive = true;
      if (now < settle_until_) {return s;}   // camera still moving
      ++looked_;
      if (good(lane)) {
        ++hits_;
        offset_sum_ += lane.offset;
      }
      if (hits_ >= cfg_.need || looked_ >= cfg_.looks) {return judge(now);}
      return s;

    case SeekPhase::Turn:
      s.drive = true;
      if (now < turn_until_) {
        s.turn = turn_dir_;
        return s;
      }
      return begin_sweep(now, last_side_);

    case SeekPhase::Follow:
      s.drive = true;
      if (lane.found) {
        last_seen_ = now;
        last_side_ = lane.offset > 0 ? -1 : 1;   // right of centre = side -1
        steer(lane, cfg_, s.forward, s.turn);
        return s;
      }
      if (now - last_seen_ <= cfg_.lost_timeout) {return s;}   // stand still, it may come back
      if (!search_) {
        phase_ = SeekPhase::Failed;
        s.event = "lane lost";
        return s;
      }
      spins_ = turns_ = 0;
      s = begin_sweep(now, last_side_);
      s.event = last_side_ > 0 ? "lane lost: searching, left first" : "lane lost: searching, right first";
      return s;
  }
  return s;
}

}  // namespace robot_vision
