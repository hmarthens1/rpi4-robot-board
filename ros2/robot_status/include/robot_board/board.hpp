// The expansion board's microcontroller (I2C bus 1, address 0x7A): 4 DC motors,
// 6 PWM servos, battery voltage. Same register protocol as Hiwonder's
// HiwonderSDK/Board.py and the Python robot_board package:
//
//   reg 0       battery: write [0], then a separate 2-byte read (mV, little-endian)
//   reg 31..34  motor 1..4 speed: write [reg, int8 -100..100]
//   reg 40      servo move: [40, count, t_lo, t_hi, id, p_lo, p_hi, ...]
//               pulse 500..2500 us, time 0..30000 ms
#pragma once

#include <array>
#include <cstdint>
#include <map>
#include <memory>
#include <optional>

#include "robot_board/i2c.hpp"

namespace robot_board {

constexpr int kI2CBus = 1;
constexpr uint8_t kBoardAddr = 0x7A;

class Board {
public:
  // polarity[m-1] = +1 or -1 per motor. Hiwonder flips motors 1 and 3 so that a
  // positive speed drives every wheel forward.
  explicit Board(std::unique_ptr<Transport> transport = nullptr,
                 std::array<int, 4> polarity = {-1, 1, -1, 1},
                 std::map<int, int> servo_offsets = {});

  // Battery voltage in mV, or nullopt. Retries: some reads come back garbled.
  std::optional<int> battery_mv(int tries = 6, int low = 3000, int high = 20000);
  std::optional<double> battery_v();

  // Motors: speed in percent, -100..100, positive = forward after polarity.
  void set_motor(int motor, int speed);
  void set_motors(const std::array<int, 4> & speeds);
  void stop();                          // all four; keeps going if one write fails
  const std::array<int, 4> & motor_speeds() const {return speeds_;}

  // Servos: pulse 500..2500 us, all listed servos move together over `ms`.
  void set_servo_pulses(const std::map<int, int> & pulses, int ms = 1000);
  void set_servo_pulse(int servo, int pulse, int ms = 1000);
  void set_servo_angle(int servo, double angle, int ms = 1000);   // 0..180 deg
  const std::array<std::optional<int>, 6> & servo_pulses() const {return pulses_;}

private:
  std::unique_ptr<Transport> io_;
  std::array<int, 4> polarity_;
  std::map<int, int> offsets_;
  std::array<int, 4> speeds_{};
  std::array<std::optional<int>, 6> pulses_{};
};

}  // namespace robot_board
