// Hiwonder I2C ultrasonic module (address 0x77, on port P7/P8/P9).
//   reg 0      distance: write [0], then a separate 2-byte read (mm, little-endian)
//   reg 2      LED mode: 0 = fixed colour, 1 = breathing
//   reg 3..8   LED 1 and LED 2 red, green, blue
#pragma once

#include <array>
#include <memory>
#include <optional>

#include "robot_board/i2c.hpp"

namespace robot_board {

constexpr uint8_t kSonarAddr = 0x77;
constexpr int kSonarMaxMm = 5000;

class Sonar {
public:
  explicit Sonar(std::unique_ptr<Transport> transport = nullptr);
  std::optional<int> distance_mm(int tries = 5);   // 1..5000 mm, or nullopt (never a made-up value)
  void set_color(int led, int r, int g, int b);    // led 0 or 1

private:
  std::unique_ptr<Transport> io_;
};

// Hiwonder mecanum mixing, motors 1..4 in percent, scaled down together past 100.
//   vx > 0 slide right, vy > 0 forward, turn > 0 counter-clockwise
std::array<int, 4> wheel_speeds(double vx, double vy, double turn);

}  // namespace robot_board
