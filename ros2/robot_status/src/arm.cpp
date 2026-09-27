#include "robot_board/arm.hpp"

#include <cmath>

namespace robot_board {

namespace {
constexpr double kPi = 3.14159265358979323846;
double deg(double r) {return r * 180.0 / kPi;}
double rad(double d) {return d * kPi / 180.0;}
// Python's round(): half to even. std::nearbyint uses the current rounding
// mode, which is round-to-nearest-even by default.
double pyround(double v, int digits = 0)
{
  const double f = std::pow(10.0, digits);
  return std::nearbyint(v * f) / f;
}
}  // namespace

std::optional<ArmIK::Angles> ArmIK::rotation_angles(double X, double Y, double Z, double Alpha) const
{
  const double theta6 = deg(std::atan2(X, Y));
  const double P_O = std::sqrt(X * X + Y * Y);
  const double CD = l4 * std::cos(rad(Alpha));
  const double PD = l4 * std::sin(rad(Alpha));
  const double AF = P_O - CD;
  const double CF = Z - l1 - PD;
  const double AC = std::sqrt(AF * AF + CF * CF);
  if (pyround(CF, 4) < -l1) {return std::nullopt;}
  if (l2 + l3 < pyround(AC, 4)) {return std::nullopt;}
  const double cos_ABC = pyround((l2 * l2 + l3 * l3 - AC * AC) / (2 * l2 * l3), 4);
  if (std::abs(cos_ABC) > 1) {return std::nullopt;}
  const double ABC = std::acos(cos_ABC);
  const double zf_flag = CF > 0 ? 1.0 : -1.0;
  const double theta4 = pyround(zf_flag * (180.0 - deg(ABC)), 4);
  const double CAF = std::acos(AF / AC);
  const double cos_BAC = pyround((AC * AC + l2 * l2 - l3 * l3) / (2 * l2 * AC), 4);
  if (std::abs(cos_BAC) > 1) {return std::nullopt;}
  const double theta5 = pyround(deg(zf_flag * (CAF - std::acos(cos_BAC))), 5);
  const double theta3 = pyround(Alpha - (theta5 + theta4), 5);
  return Angles{theta3, theta4, theta5, theta6};
}

std::optional<ArmSolution> ArmIK::to_pulses(const Angles & a, double alpha)
{
  constexpr double param = 2000.0 / 180.0, mid = 1500.0;
  const auto in_range = [](int p) {return p >= 500 && p <= 2500;};
  const int s3 = static_cast<int>(pyround(a.theta3 * param + mid));
  if (!in_range(s3)) {return std::nullopt;}
  const int s4 = static_cast<int>(pyround(-a.theta4 * param + mid));
  if (!in_range(s4)) {return std::nullopt;}
  const int s5 = static_cast<int>(pyround(mid + (90 - a.theta5) * param));
  if (!in_range(s5)) {return std::nullopt;}
  const int s6 = static_cast<int>(pyround(a.theta6 * param + mid));
  if (!in_range(s6)) {return std::nullopt;}
  return ArmSolution{s3, s4, s5, s6, alpha};
}

std::optional<ArmSolution> ArmIK::pitch_range(double x, double y, double z, double alpha1,
                                              double alpha2, double da) const
{
  if (alpha1 >= alpha2) {da = -da;}
  // numpy.arange(alpha1, alpha2, da): n = ceil((alpha2 - alpha1) / da) values
  const long n = static_cast<long>(std::ceil((alpha2 - alpha1) / da));
  for (long i = 0; i < n; ++i) {
    const double alpha = alpha1 + i * da;
    if (auto angles = rotation_angles(x, y, z, alpha)) {
      if (auto s = to_pulses(*angles, alpha)) {return s;}
    }
  }
  return std::nullopt;
}

std::optional<ArmSolution> ArmIK::solve(double x, double y, double z, double alpha,
                                        double alpha1, double alpha2) const
{
  auto r1 = pitch_range(x, y, z, alpha, alpha1);
  auto r2 = pitch_range(x, y, z, alpha, alpha2);
  if (r1) {
    if (r2 && std::abs(r2->alpha - alpha) < std::abs(r1->alpha - alpha)) {return r2;}
    return r1;
  }
  return r2;
}

const std::map<std::string, ArmPose> & arm_poses()
{
  // gripper, wrist, elbow, shoulder, base
  static const std::map<std::string, ArmPose> poses = {
    {"stand", {1500, 695, 2415, 780, 1500}},    // stand.d6a
    {"rest", {1500, 590, 2500, 700, 1500}},     // start/end frame of the lab action groups
    {"ready", {2500, 800, 2000, 2100, 1500}},   // initMove in Project 2 / demo_pickNplace
    {"center", {1500, 1500, 1500, 1500, 1500}}, // MasterPi_PC_Software "Reset"
  };
  return poses;
}

}  // namespace robot_board
