// Protocol tests: the C++ driver must send exactly the bytes that Hiwonder's
// HiwonderSDK/Board.py sends (the same expectations as tests/test_board.py,
// which were cross-checked against Board.py itself). No hardware needed.
#include <gtest/gtest.h>

#include <deque>
#include <stdexcept>
#include <system_error>

#include "robot_board/board.hpp"
#include "robot_board/sonar.hpp"

using robot_board::Board;
using robot_board::Bytes;
using robot_board::Sonar;

namespace {

struct Record {
  std::vector<Bytes> writes;
  std::deque<int> reads;          // values to return (little-endian 16 bit); -1 = I/O error
};

class FakeBus : public robot_board::Transport {
public:
  explicit FakeBus(Record & r) : r_(r) {}
  void write(const Bytes & data) override {r_.writes.push_back(data);}
  Bytes read(size_t) override
  {
    int v = 12220;
    if (!r_.reads.empty()) {v = r_.reads.front(); r_.reads.pop_front();}
    if (v < 0) {throw std::system_error(EIO, std::generic_category(), "fake");}
    return {static_cast<uint8_t>(v & 0xFF), static_cast<uint8_t>(v >> 8)};
  }

private:
  Record & r_;
};

Board make_board(Record & r, std::map<int, int> offsets = {})
{
  return Board(std::make_unique<FakeBus>(r), {-1, 1, -1, 1}, offsets);
}

// Board.setMotor: motors 1 and 3 negated, reg 31+i, int8
Bytes hiwonder_motor(int index, int speed)
{
  if (index != 2 && index != 4) {speed = -speed;}
  speed = std::max(-100, std::min(100, speed));
  return {static_cast<uint8_t>(31 + index - 1), static_cast<uint8_t>(static_cast<int8_t>(speed))};
}

// Board.setPWMServoPulse
Bytes hiwonder_servo(int id, int pulse, int ms, int deviation = 0)
{
  pulse = std::max(500, std::min(2500, pulse + deviation));
  ms = std::max(0, std::min(30000, ms));
  return {40, 1, static_cast<uint8_t>(ms & 0xFF), static_cast<uint8_t>(ms >> 8),
    static_cast<uint8_t>(id), static_cast<uint8_t>(pulse & 0xFF), static_cast<uint8_t>(pulse >> 8)};
}

}  // namespace

TEST(Motors, SameBytesAsHiwonder) {
  for (int m = 1; m <= 4; ++m) {
    for (int s : {-120, -100, -37, -1, 0, 1, 50, 100, 150}) {
      Record r;
      make_board(r).set_motor(m, s);
      ASSERT_EQ(r.writes.size(), 1u);
      EXPECT_EQ(r.writes[0], hiwonder_motor(m, s)) << "motor " << m << " speed " << s;
    }
  }
}

TEST(Motors, StopWritesAllFour) {
  Record r;
  make_board(r).stop();
  EXPECT_EQ(r.writes, (std::vector<Bytes>{{31, 0}, {32, 0}, {33, 0}, {34, 0}}));
}

TEST(Motors, BadMotor) {
  Record r;
  auto b = make_board(r);
  EXPECT_THROW(b.set_motor(5, 10), std::invalid_argument);
}

TEST(Servos, SameBytesAsHiwonder) {
  for (int id : {1, 6}) {
    for (auto [p, ms] : std::vector<std::pair<int, int>>{{1500, 1000}, {400, 20}, {2600, 40000}, {500, 0}}) {
      Record r;
      make_board(r).set_servo_pulse(id, p, ms);
      EXPECT_EQ(r.writes.at(0), hiwonder_servo(id, p, ms));
    }
  }
}

TEST(Servos, OffsetAndMissingOffset) {
  Record r;
  auto b = make_board(r, {{5, -64}});
  b.set_servo_pulse(5, 1500, 500);
  b.set_servo_pulse(2, 1500, 1000);    // Deviation.yaml has no '2': Board.py raises; here 0
  EXPECT_EQ(r.writes.at(0), hiwonder_servo(5, 1500, 500, -64));
  EXPECT_EQ(r.writes.at(1), hiwonder_servo(2, 1500, 1000));
}

TEST(Servos, SeveralAtOnce) {
  Record r;
  make_board(r).set_servo_pulses({{1, 1000}, {3, 2000}}, 800);
  EXPECT_EQ(r.writes.at(0), (Bytes{40, 2, 0x20, 0x03, 1, 0xE8, 0x03, 3, 0xD0, 0x07}));
}

TEST(Servos, Angle) {
  Record r;
  make_board(r).set_servo_angle(1, 90, 0);
  EXPECT_EQ(r.writes.at(0), hiwonder_servo(1, 1500, 0));
}

TEST(Battery, WriteThenSeparateRead) {
  Record r;
  EXPECT_EQ(make_board(r).battery_mv(), 12220);
  EXPECT_EQ(r.writes, (std::vector<Bytes>{{0}}));
}

TEST(Battery, RetriesGarbledAndFailedReads) {
  Record r;
  r.reads = {65470, -1, 231, 12250};
  EXPECT_EQ(make_board(r).battery_mv(), 12250);
}

TEST(Battery, GivesUp) {
  Record r;
  r.reads = std::deque<int>(10, -1);
  EXPECT_FALSE(make_board(r).battery_v().has_value());
}

TEST(Sonar, Distance) {
  Record r;
  r.reads = {1093};
  EXPECT_EQ(Sonar(std::make_unique<FakeBus>(r)).distance_mm(), 1093);
}

TEST(Sonar, GarbageIsRetriedNeverReportedAsFreeSpace) {
  Record r;
  r.reads = {65439, 65439, 327};          // 0xFF9F glitches seen on robot01, then a real reading
  EXPECT_EQ(Sonar(std::make_unique<FakeBus>(r)).distance_mm(), 327);
  Record r2;
  r2.reads = std::deque<int>(10, 65439);  // only garbage: no reading, not "5 m"
  EXPECT_FALSE(Sonar(std::make_unique<FakeBus>(r2)).distance_mm().has_value());
}

TEST(Sonar, PresentEvenWhenTheReadingIsGarbage) {
  Record r;
  r.reads = {65439};
  EXPECT_TRUE(Sonar(std::make_unique<FakeBus>(r)).present());
  Record r2;
  r2.reads = std::deque<int>(10, -1);        // no answer at all
  EXPECT_FALSE(Sonar(std::make_unique<FakeBus>(r2)).present());
}

TEST(Sonar, Color) {
  Record r;
  Sonar(std::make_unique<FakeBus>(r)).set_color(1, 10, 20, 30);
  EXPECT_EQ(r.writes, (std::vector<Bytes>{{2, 0}, {6, 10}, {7, 20}, {8, 30}}));
}

TEST(Mecanum, MatchesHiwonderMixing) {
  EXPECT_EQ(robot_board::wheel_speeds(0, 50, 0), (std::array<int, 4>{50, 50, 50, 50}));
  EXPECT_EQ(robot_board::wheel_speeds(40, 0, 0), (std::array<int, 4>{40, -40, -40, 40}));
  EXPECT_EQ(robot_board::wheel_speeds(0, 0, 30), (std::array<int, 4>{-30, 30, -30, 30}));
  EXPECT_EQ(robot_board::wheel_speeds(100, 100, 0), (std::array<int, 4>{100, 0, 0, 100}));
}
