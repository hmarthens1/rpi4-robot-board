// I2C transport for the expansion board: raw writes and reads on /dev/i2c-N,
// one transfer each (I2C_RDWR), with the SDK's retry, and a lock shared by
// every process that uses the board.
//
// A register read on the board's microcontroller is two transfers - write the
// register number, then read - and must not be a combined write+read (that
// returns garbage). If another process writes in between, the read returns the
// wrong register. So every transfer takes the cross-process lock in
// /tmp/robot_board_i2c.lock (the same file the Python robot_board package
// uses, so C++ nodes and Python scripts exclude each other too), and
// multi-transfer reads hold it throughout via transaction().
#pragma once

#include <cstdint>
#include <memory>
#include <string>
#include <vector>

namespace robot_board {

using Bytes = std::vector<uint8_t>;

// What the driver needs from a bus. Tests plug in a fake.
class Transport {
public:
  virtual ~Transport() = default;
  virtual void write(const Bytes & data) = 0;
  virtual Bytes read(size_t n) = 0;
  // Hold the bus for several transfers. Default: no locking (tests).
  struct Guard { virtual ~Guard() = default; };
  virtual std::unique_ptr<Guard> transaction() { return std::make_unique<Guard>(); }
};

class I2CTransport : public Transport {
public:
  static constexpr const char * kLockFile = "/tmp/robot_board_i2c.lock";

  I2CTransport(int bus, uint8_t addr, int retries = 2);
  ~I2CTransport() override;
  I2CTransport(const I2CTransport &) = delete;
  I2CTransport & operator=(const I2CTransport &) = delete;

  void write(const Bytes & data) override;
  Bytes read(size_t n) override;
  std::unique_ptr<Guard> transaction() override;

private:
  class Lock;
  void rdwr(bool is_read, uint8_t * buf, uint16_t len);
  std::string path_;
  uint8_t addr_;
  int retries_;
  int fd_ = -1;
  int held_ = 0;     // nesting depth of transaction() in this object
};

}  // namespace robot_board
