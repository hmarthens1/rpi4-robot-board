#include "robot_board/peripherals.hpp"

#include <gpiod.hpp>

#include <chrono>
#include <cstring>
#include <stdexcept>
#include <string>
#include <thread>

extern "C" {
#include "ws2811.h"
}

namespace robot_board {

// ------------------------------------------------------------------- RGB
struct Rgb::Impl {
  ws2811_t strip{};
};

Rgb::Rgb(int brightness)
: impl_(std::make_unique<Impl>())
{
  // Same settings as HiwonderSDK / the Python driver: 800 kHz, DMA 10,
  // GPIO12 (PWM0), 2 LEDs, GRB order.
  ws2811_t & s = impl_->strip;
  s.freq = WS2811_TARGET_FREQ;
  s.dmanum = 10;
  s.channel[0].gpionum = 12;
  s.channel[0].count = 2;
  s.channel[0].invert = 0;
  s.channel[0].brightness = static_cast<uint8_t>(brightness);
  s.channel[0].strip_type = WS2811_STRIP_GRB;
  s.channel[1].gpionum = 0;
  s.channel[1].count = 0;
  ws2811_return_t ret = ws2811_init(&s);
  if (ret != WS2811_SUCCESS) {
    throw std::runtime_error(std::string("RGB LEDs: ") + ws2811_get_return_t_str(ret));
  }
  off();
}

Rgb::~Rgb()
{
  if (impl_) {
    try {off();} catch (...) {}
    ws2811_fini(&impl_->strip);
  }
}

void Rgb::fill(int r, int g, int b)
{
  auto clip = [](int v) {return static_cast<uint32_t>(v < 0 ? 0 : v > 255 ? 255 : v);};
  const uint32_t color = (clip(r) << 16) | (clip(g) << 8) | clip(b);
  for (int i = 0; i < impl_->strip.channel[0].count; ++i) {
    impl_->strip.channel[0].leds[i] = color;
  }
  ws2811_return_t ret = ws2811_render(&impl_->strip);
  if (ret != WS2811_SUCCESS) {
    throw std::runtime_error(std::string("RGB render: ") + ws2811_get_return_t_str(ret));
  }
}

// ------------------------------------------------------------------ GPIO
namespace {
constexpr unsigned kBuzzer = 6, kLed1 = 16, kLed2 = 26, kKey1 = 13, kKey2 = 23;
constexpr const char * kConsumer = "robot_board";
}  // namespace

struct Gpio::Impl {
  gpiod::chip chip{"gpiochip0"};
  gpiod::line buzzer, led1, led2, key1, key2;
};

Gpio::Gpio()
: impl_(std::make_unique<Impl>())
{
  auto out = [&](unsigned pin, int initial) {
    auto line = impl_->chip.get_line(pin);
    line.request({kConsumer, gpiod::line_request::DIRECTION_OUTPUT, 0}, initial);
    return line;
  };
  auto in = [&](unsigned pin) {
    auto line = impl_->chip.get_line(pin);
    line.request({kConsumer, gpiod::line_request::DIRECTION_INPUT,
                  gpiod::line_request::FLAG_BIAS_PULL_UP});
    return line;
  };
  impl_->buzzer = out(kBuzzer, 0);
  impl_->led1 = out(kLed1, 1);    // active low: 1 = off
  impl_->led2 = out(kLed2, 1);
  impl_->key1 = in(kKey1);
  impl_->key2 = in(kKey2);
}

Gpio::~Gpio()
{
  try {
    impl_->buzzer.set_value(0);
    impl_->led1.set_value(1);
    impl_->led2.set_value(1);
  } catch (...) {
  }
  // gpiod::line releases the request when it is destroyed.
}

void Gpio::buzzer(bool on) {impl_->buzzer.set_value(on ? 1 : 0);}

void Gpio::beep(double seconds, int times, double gap)
{
  using std::chrono::duration;
  for (int i = 0; i < times; ++i) {
    buzzer(true);
    std::this_thread::sleep_for(duration<double>(seconds));
    buzzer(false);
    if (i < times - 1) {std::this_thread::sleep_for(duration<double>(gap));}
  }
}

void Gpio::led(int n, bool on)
{
  if (n != 1 && n != 2) {throw std::invalid_argument("led must be 1 or 2");}
  (n == 1 ? impl_->led1 : impl_->led2).set_value(on ? 0 : 1);
}

bool Gpio::key_pressed(int n)
{
  if (n != 1 && n != 2) {throw std::invalid_argument("key must be 1 or 2");}
  return (n == 1 ? impl_->key1 : impl_->key2).get_value() == 0;
}

}  // namespace robot_board
