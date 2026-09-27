// The 4-DOF PWM servo arm + gripper, as used in the MSE112 labs and
// MasterPi/ArmIK (InverseKinematics.py, ArmMoveIK.py). A port of that code,
// cross-checked against the Python original (test/test_arm.cpp).
//
//   servo 1  gripper      ~2000-2500 open, ~1000-1500 closed
//   servo 2  (fan port - not part of the arm)
//   servo 3  wrist pitch  1500 + theta3 * 11.11
//   servo 4  elbow        1500 - theta4 * 11.11   (inverted)
//   servo 5  shoulder     1500 + (90 - theta5) * 11.11
//   servo 6  base yaw     1500 + theta6 * 11.11
//
// Frame (cm): origin under the base-yaw axis on the ground, +y forward,
// +x right, +z up. Pitch alpha (deg) = gripper angle to the horizontal,
// 0 level, negative pointing down. Links l1..l4 = 8.0, 6.0, 6.2, 10.0 cm.
#pragma once

#include <map>
#include <optional>
#include <string>

namespace robot_board {

struct ArmSolution {
  int servo3, servo4, servo5, servo6;   // pulses (us), before calibration offsets
  double alpha;                          // the pitch actually used (deg)
};

class ArmIK {
public:
  double l1 = 8.0, l2 = 6.0, l3 = 6.2, l4 = 10.0;   // cm

  struct Angles {double theta3, theta4, theta5, theta6;};
  // IK.getRotationAngle: joint angles for (x, y, z) at pitch alpha, or nullopt.
  std::optional<Angles> rotation_angles(double x, double y, double z, double alpha) const;
  // ArmIK.transformAngleAdaptArm: angles -> pulses 500..2500, or nullopt if out of range.
  static std::optional<ArmSolution> to_pulses(const Angles & a, double alpha);
  // ArmIK.setPitchRange: first solution stepping 1 deg from alpha1 towards alpha2
  // (alpha2 excluded, like numpy.arange).
  std::optional<ArmSolution> pitch_range(double x, double y, double z, double alpha1,
                                         double alpha2, double da = 1.0) const;
  // ArmIK.setPitchRangeMoving (without moving): the solution whose pitch is
  // closest to alpha, searching alpha->alpha1 and alpha->alpha2.
  std::optional<ArmSolution> solve(double x, double y, double z, double alpha,
                                   double alpha1, double alpha2) const;
};

// Named arm poses, raw pulses for servos 1, 3, 4, 5, 6 (from the lab code and
// the stand.d6a action group).
struct ArmPose {int s1, s3, s4, s5, s6;};
const std::map<std::string, ArmPose> & arm_poses();

constexpr int kGripperOpen = 2000;
constexpr int kGripperClosed = 1500;

}  // namespace robot_board
