// Parts of the expansion board wired straight to the Pi's GPIO header (BCM
// numbering, from the board's schematic):
//
//   GPIO12  2x WS2812 RGB LEDs   -> Rgb     (rpi_ws281x: DMA + PWM, needs root)
//   GPIO6   buzzer (transistor)   -> Buzzer  (libgpiod)
//   GPIO16  LED1, active low      -> Leds
//   GPIO26  LED2, active low
//   GPIO13  Key1 to GND           -> Keys
//   GPIO23  Key2 to GND
#pragma once

#include <memory>

namespace gpiod {class chip; class line;}

namespace robot_board {

class Rgb {
public:
  Rgb(int brightness = 120);      // throws if not root / hardware unavailable
  ~Rgb();
  Rgb(const Rgb &) = delete;
  Rgb & operator=(const Rgb &) = delete;
  void fill(int r, int g, int b);
  void off() {fill(0, 0, 0);}

private:
  struct Impl;
  std::unique_ptr<Impl> impl_;
};

// Buzzer, LED1/LED2 and Key1/Key2 through /dev/gpiochip0 (no root needed for
// a user in the group that owns it).
class Gpio {
public:
  Gpio();
  ~Gpio();                        // buzzer off, LEDs off, lines released
  void buzzer(bool on);
  void beep(double seconds, int times, double gap = 0.15);
  void led(int n, bool on);       // n = 1 or 2
  bool key_pressed(int n);        // n = 1 or 2

private:
  struct Impl;
  std::unique_ptr<Impl> impl_;
};

}  // namespace robot_board
