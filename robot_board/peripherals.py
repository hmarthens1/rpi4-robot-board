"""
The parts of the expansion board wired straight to the Pi's GPIO header
(from the board's schematic, BCM numbering):

    GPIO12  pin 32  2x WS2812 RGB LEDs      -> RGB   (needs sudo: rpi_ws281x uses /dev/mem)
    GPIO6   pin 31  buzzer, via a transistor -> Buzzer
    GPIO13  pin 33  Key1, button to GND      -> Keys
    GPIO23  pin 16  Key2, button to GND      -> Keys
    GPIO16  pin 36  LED1, active low          -> Leds
    GPIO26  pin 37  LED2, active low          -> Leds

Buzzer, Keys and Leds use RPi.GPIO (Ubuntu package python3-rpi.gpio) and work
without sudo for a user in the 'dialout' group (/dev/gpiomem).
"""
import time

_gpio = None


def _GPIO():
    """Import and set up RPi.GPIO on first use, so importing this module is cheap."""
    global _gpio
    if _gpio is None:
        import RPi.GPIO as GPIO
        GPIO.setwarnings(False)
        GPIO.setmode(GPIO.BCM)
        _gpio = GPIO
    return _gpio


PIN_RGB = 12
PIN_BUZZER = 6
PIN_KEY1, PIN_KEY2 = 13, 23
PIN_LED1, PIN_LED2 = 16, 26


class RGB:
    """The two WS2812 LEDs. Needs root."""

    COUNT = 2

    def __init__(self, brightness=120):
        from rpi_ws281x import PixelStrip
        # count, pin, 800 kHz, DMA channel 10, not inverted, brightness, PWM channel 0
        self.strip = PixelStrip(self.COUNT, PIN_RGB, 800000, 10, False, brightness, 0)
        self.strip.begin()
        self.off()

    def set(self, index, r, g, b, show=True):
        from rpi_ws281x import Color
        self.strip.setPixelColor(index, Color(int(r), int(g), int(b)))
        if show:
            self.strip.show()

    def fill(self, r, g, b):
        for i in range(self.COUNT):
            self.set(i, r, g, b, show=False)
        self.strip.show()

    def off(self):
        self.fill(0, 0, 0)


class Buzzer:
    def __init__(self):
        _GPIO().setup(PIN_BUZZER, _GPIO().OUT, initial=0)

    def on(self):
        _GPIO().output(PIN_BUZZER, 1)

    def off(self):
        _GPIO().output(PIN_BUZZER, 0)

    def beep(self, seconds=0.1, times=1, gap=0.1):
        for i in range(times):
            self.on()
            time.sleep(seconds)
            self.off()
            if i < times - 1:
                time.sleep(gap)


class Keys:
    """Key1 and Key2. pressed(1) / pressed(2) -> True while held down."""

    PINS = {1: PIN_KEY1, 2: PIN_KEY2}

    def __init__(self):
        for pin in self.PINS.values():
            _GPIO().setup(pin, _GPIO().IN, pull_up_down=_GPIO().PUD_UP)

    def pressed(self, key):
        return _GPIO().input(self.PINS[key]) == 0


class Leds:
    """LED1 and LED2 (active low: the pin pulls the LED's cathode to ground)."""

    PINS = {1: PIN_LED1, 2: PIN_LED2}

    def __init__(self):
        for pin in self.PINS.values():
            _GPIO().setup(pin, _GPIO().OUT, initial=1)   # 1 = off

    def set(self, led, on):
        _GPIO().output(self.PINS[led], 0 if on else 1)


def cleanup():
    """Release the GPIO pins (buzzer off, LEDs back to inputs)."""
    if _gpio is not None:
        _gpio.cleanup()
