#include "robot_board/i2c.hpp"

#include <fcntl.h>
#include <linux/i2c-dev.h>
#include <linux/i2c.h>
#include <sys/file.h>
#include <sys/ioctl.h>
#include <sys/stat.h>
#include <unistd.h>

#include <cerrno>
#include <chrono>
#include <system_error>
#include <thread>

namespace robot_board {

// flock on the shared lock file. Created world-writable so root (command node)
// and the normal user (status node, scripts) can all take it.
class I2CTransport::Lock : public Transport::Guard {
public:
  explicit Lock(I2CTransport & owner) : owner_(owner) {
    if (owner_.held_++ > 0) {
      return;                         // already held by this object: nested
    }
    mode_t old = umask(0);
    fd_ = open(kLockFile, O_RDWR | O_CREAT | O_CLOEXEC, 0666);
    umask(old);
    if (fd_ < 0 || flock(fd_, LOCK_EX) != 0) {
      int err = errno;
      if (fd_ >= 0) {close(fd_);}
      owner_.held_--;
      throw std::system_error(err, std::generic_category(), "I2C lock");
    }
  }
  ~Lock() override {
    if (--owner_.held_ == 0 && fd_ >= 0) {
      flock(fd_, LOCK_UN);
      close(fd_);
    }
  }

private:
  I2CTransport & owner_;
  int fd_ = -1;
};

I2CTransport::I2CTransport(int bus, uint8_t addr, int retries)
: path_("/dev/i2c-" + std::to_string(bus)), addr_(addr), retries_(retries)
{
  fd_ = open(path_.c_str(), O_RDWR | O_CLOEXEC);
  if (fd_ < 0) {
    throw std::system_error(errno, std::generic_category(), "open " + path_);
  }
}

I2CTransport::~I2CTransport()
{
  if (fd_ >= 0) {close(fd_);}
}

std::unique_ptr<Transport::Guard> I2CTransport::transaction()
{
  return std::make_unique<Lock>(*this);
}

void I2CTransport::rdwr(bool is_read, uint8_t * buf, uint16_t len)
{
  i2c_msg msg{};
  msg.addr = addr_;
  msg.flags = is_read ? I2C_M_RD : 0;
  msg.len = len;
  msg.buf = buf;
  i2c_rdwr_ioctl_data data{&msg, 1};
  for (int attempt = 0;; ++attempt) {
    if (ioctl(fd_, I2C_RDWR, &data) >= 0) {
      return;
    }
    if (attempt >= retries_) {
      throw std::system_error(errno, std::generic_category(), "I2C transfer");
    }
    std::this_thread::sleep_for(std::chrono::milliseconds(2));
  }
}

void I2CTransport::write(const Bytes & data)
{
  auto guard = transaction();   // a lone write also waits for other processes' reads
  Bytes copy = data;            // the kernel API wants a mutable buffer
  rdwr(false, copy.data(), static_cast<uint16_t>(copy.size()));
}

Bytes I2CTransport::read(size_t n)
{
  auto guard = transaction();
  Bytes buf(n);
  rdwr(true, buf.data(), static_cast<uint16_t>(n));
  return buf;
}

}  // namespace robot_board
