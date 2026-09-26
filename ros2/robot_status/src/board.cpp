#include "robot_board/board.hpp"

#include <algorithm>
#include <cmath>
#include <stdexcept>
#include <string>
#include <system_error>

namespace robot_board {

namespace {
constexpr uint8_t kRegBattery = 0;
constexpr uint8_t kRegMotor = 31;
constexpr uint8_t kRegServoMove = 40;
constexpr int kPulseMin = 500, kPulseMax = 2500, kMoveTimeMax = 30000;

template<typename T>
T clamp(T v, T lo, T hi) {return std::max(lo, std::min(hi, v));}
}  // namespace

Board::Board(std::unique_ptr<Transport> transport, std::array<int, 4> polarity,
             std::map<int, int> servo_offsets)
: io_(transport ? std::move(transport) : std::make_unique<I2CTransport>(kI2CBus, kBoardAddr)),
  polarity_(polarity), offsets_(std::move(servo_offsets)) {}

std::optional<int> Board::battery_mv(int tries, int low, int high)
{
  for (int i = 0; i < tries; ++i) {
    try {
      Bytes raw;
      {
        auto guard = io_->transaction();   // register write + read, nothing in between
        io_->write({kRegBattery});
        raw = io_->read(2);
      }
      int mv = raw[0] | (raw[1] << 8);
      if (mv >= low && mv <= high) {
        return mv;
      }
    } catch (const std::system_error &) {
      // no answer this time: try again
    }
  }
  return std::nullopt;
}

std::optional<double> Board::battery_v()
{
  auto mv = battery_mv();
  if (!mv) {return std::nullopt;}
  return std::round(*mv / 10.0) / 100.0;   // 2 decimals
}

void Board::set_motor(int motor, int speed)
{
  if (motor < 1 || motor > 4) {
    throw std::invalid_argument("motor must be 1-4, not " + std::to_string(motor));
  }
  speed = clamp(speed, -100, 100);
  int8_t raw = static_cast<int8_t>(speed * polarity_[motor - 1]);
  io_->write({static_cast<uint8_t>(kRegMotor + motor - 1), static_cast<uint8_t>(raw)});
  speeds_[motor - 1] = speed;
}

void Board::set_motors(const std::array<int, 4> & speeds)
{
  for (int m = 1; m <= 4; ++m) {
    set_motor(m, speeds[m - 1]);
  }
}

void Board::stop()
{
  std::exception_ptr error;
  for (int m = 1; m <= 4; ++m) {
    try {
      set_motor(m, 0);
    } catch (...) {
      error = std::current_exception();
    }
  }
  if (error) {std::rethrow_exception(error);}
}

void Board::set_servo_pulses(const std::map<int, int> & pulses, int ms)
{
  if (pulses.empty()) {return;}
  ms = clamp(ms, 0, kMoveTimeMax);
  Bytes buf{kRegServoMove, static_cast<uint8_t>(pulses.size()),
            static_cast<uint8_t>(ms & 0xFF), static_cast<uint8_t>(ms >> 8)};
  for (const auto & [servo, pulse_in] : pulses) {
    if (servo < 1 || servo > 6) {
      throw std::invalid_argument("servo must be 1-6, not " + std::to_string(servo));
    }
    auto off = offsets_.find(servo);
    int pulse = clamp(pulse_in + (off == offsets_.end() ? 0 : off->second), kPulseMin, kPulseMax);
    buf.insert(buf.end(), {static_cast<uint8_t>(servo), static_cast<uint8_t>(pulse & 0xFF),
                           static_cast<uint8_t>(pulse >> 8)});
    pulses_[servo - 1] = pulse;
  }
  io_->write(buf);
}

void Board::set_servo_pulse(int servo, int pulse, int ms)
{
  set_servo_pulses({{servo, pulse}}, ms);
}

void Board::set_servo_angle(int servo, double angle, int ms)
{
  angle = clamp(angle, 0.0, 180.0);
  set_servo_pulse(servo, static_cast<int>(kPulseMin + angle * (kPulseMax - kPulseMin) / 180.0), ms);
}

}  // namespace robot_board
