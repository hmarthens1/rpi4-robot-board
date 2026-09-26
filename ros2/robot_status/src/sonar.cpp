#include <algorithm>
#include <array>
#include <cmath>
#include <stdexcept>
#include <system_error>

#include "robot_board/board.hpp"
#include "robot_board/sonar.hpp"

namespace robot_board {

Sonar::Sonar(std::unique_ptr<Transport> transport)
: io_(transport ? std::move(transport) : std::make_unique<I2CTransport>(kI2CBus, kSonarAddr)) {}

std::optional<int> Sonar::distance_mm(int tries)
{
  // The module sometimes answers 0xFF9F-style garbage. Capping that at 5000
  // would report "5 m of free space" - so anything outside 1..5000 mm is an
  // invalid reading: retry, and give up with "no reading" rather than guess.
  for (int i = 0; i < tries; ++i) {
    try {
      Bytes raw;
      {
        auto guard = io_->transaction();
        io_->write({0});
        raw = io_->read(2);
      }
      const int mm = raw[0] | (raw[1] << 8);
      if (mm >= 1 && mm <= kSonarMaxMm) {
        return mm;
      }
    } catch (const std::system_error &) {
    }
  }
  return std::nullopt;
}

void Sonar::set_color(int led, int r, int g, int b)
{
  if (led != 0 && led != 1) {throw std::invalid_argument("led must be 0 or 1");}
  io_->write({2, 0});
  const uint8_t base = led == 0 ? 3 : 6;
  const int rgb[3] = {r, g, b};
  for (int i = 0; i < 3; ++i) {
    io_->write({static_cast<uint8_t>(base + i), static_cast<uint8_t>(rgb[i] & 0xFF)});
  }
}

std::array<int, 4> wheel_speeds(double vx, double vy, double turn)
{
  std::array<double, 4> s{vy + vx - turn, vy - vx + turn, vy - vx - turn, vy + vx + turn};
  double biggest = 0;
  for (double v : s) {biggest = std::max(biggest, std::abs(v));}
  if (biggest > 100) {
    for (double & v : s) {v = v * 100 / biggest;}   // keep the direction
  }
  std::array<int, 4> out{};
  for (int i = 0; i < 4; ++i) {out[i] = static_cast<int>(std::lround(s[i]));}
  return out;
}

}  // namespace robot_board
