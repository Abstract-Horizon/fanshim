#!/usr/bin/env python3
#
# Based on:
# https://github.com/jane-t/rpi-fanshim
#
import argparse
import atexit
import colorsys
import os
import time
import subprocess
from typing import Optional

import RPi.GPIO as GPIO

import logging


FAN_PIN = 18
LED_DAT_PIN = 15
LED_CLK_PIN = 14
DEFAULT_PID_KP = 0.95
DEFAULT_PID_KI = 0.05
DEFAULT_PID_GAIN = 1.00
DEFAULT_PWM_SPEED = 100
DEFAULT_PID_MIN_PWM_SPEED = 40
DEFAULT_PID_MAX_PWM_SPEED = 100

logger = logging.getLogger()

parser = argparse.ArgumentParser()
parser.add_argument("-q", "--quiet", action="store_true", default=False, help="Set debugging level to quiet")
parser.add_argument("-v", "--verbose", action="store_true", default=False, help="Set debugging level to verbose")

parser.add_argument("--off-threshold", type=float, default=55.0, help="Temperature threshold in degrees C to enable fan")
parser.add_argument("--on-threshold", type=float, default=65.0, help="Temperature threshold in degrees C to disable fan")
parser.add_argument("--delay", type=float, default=2.0, help="Delay, in seconds, between temperature readings")
# parser.add_argument("--preempt", action="store_true", default=False, help="Monitor CPU frequency and activate cooling premptively")
# parser.add_argument("--verbose", action="store_true", default=False, help="Output temp and fan status messages")
# parser.add_argument("--no-button", action="store_true", default=False, help="Disable button input")
parser.add_argument("--no-led", action="store_true", default=False, help="Disable LED control")
parser.add_argument("--brightness", type=float, default=31.0, help="LED brightness, from 0 to 255")

pwn_or_not = parser.add_mutually_exclusive_group()
pwn_or_not.add_argument("--pwm", action="store_true", help=f"If selected fan is going to be driven with PWM at {DEFAULT_PWM_SPEED}% of speed")
pwn_or_not.add_argument("--pwm-speed", default=-1, help="If selected fan is going to be driven with PWM at given speed betwen 0-100")
pwn_or_not.add_argument("--pid", action="store_true", help="If selected fan will be driven by PID algorithm")
pwn_or_not.add_argument("--on-off", action="store_true", help="Default if nothing else selected. No need to specify.")

parser.add_argument("--pid-kp", type=float, default=DEFAULT_PID_KP, help=f"PID proportional parameter. Used only if --pid parameter is set. Default {DEFAULT_PID_KP}")
parser.add_argument("--pid-ki", type=float, default=DEFAULT_PID_KI, help=f"PID integral parameter. Used only if --pid parameter is set. Default {DEFAULT_PID_KI}")
parser.add_argument("--pid-gain", type=float, default=DEFAULT_PID_GAIN, help=f"PID integral parameter. Used only if --pid parameter is set. Default {DEFAULT_PID_GAIN}")
parser.add_argument("--pid-min-pwm", type=float, default=DEFAULT_PID_MIN_PWM_SPEED, help=f"PWM min speed when using PID. Used only if --pid parameter is set. Default {DEFAULT_PID_MIN_PWM_SPEED}")
parser.add_argument("--pid-max-pwm", type=float, default=DEFAULT_PID_MAX_PWM_SPEED, help=f"PWM max speed when using PID. Used only if --pid parameter is set. Default {DEFAULT_PID_MAX_PWM_SPEED}")

parser.add_argument("--fan-pin", type=int, default=FAN_PIN, help=f"Fan pin, default {FAN_PIN}")
parser.add_argument("--led-clk-pin", type=int, default=LED_DAT_PIN, help=f"LED CLK pin, default {LED_DAT_PIN}")
parser.add_argument("--led-data-pin", type=int, default=LED_CLK_PIN, help=f"LED DATA pin, default {LED_CLK_PIN}")


args = parser.parse_args()

level = logging.INFO
if args.quiet:
    level = logging.WARNING
elif args.verbose:
    level = logging.DEBUG

logging.basicConfig(level=level, format="%(message)s")
logger.setLevel(level)

GPIO.setwarnings(False)
GPIO.setmode(GPIO.BCM)


def install(force: bool = False) -> None:
    this_file = __file__
    parent_path = os.path.dirname(this_file)
    service_name = os.path.split(this_file)[-1][:-3]

    logger.info(f"Installing daemon from this file {this_file}.")
    logger.info(f"Will use user 'root' to run daemon.")

    if not os.path.exists("/etc/systemd/system/"):
        logger.error("ERROR: It seems that this system doesn't have systemd - directory '/etc/systemd/system/' is missing.")
        logger.error("ERROR: No daemon will be installed.")
    else:
        if os.path.exists(f"/etc/systemd/system/{service_name}.service") and not force:
            logger.error(f"ERROR: '/etc/systemd/system/{service_name}.service' already exists. Use --force switch if you want to override it.")
        else:
            print(f"Installing /etc/systemd/system/{service_name}.service")
            try:
                with open(f"/etc/systemd/system/{service_name}.service", "wb") as f:
                    f.write(f"""
[Unit]
Description=Fanshim Service
After=rsyslog.service

[Service]
WorkingDirectory={parent_path}
ExecStart=/usr/bin/python3 {this_file} --on-threshold 75 --off-threshold 60 --delay 2 --brightness 31 --pwm-speed 80 
Restart=on-failure
StandardOutput=syslog
StandardError=syslog

[Install]
WantedBy=multi-user.target
""".encode("UTF-8"))

                logger.info(f"Running: systemctl enable {service_name}.service")
                os.system(f"systemctl enable {service_name}.service")

                logger.info("Daemon successfully installed. Do following to run it:")
                logger.info(f"service {service_name} start")
            except PermissionError:
                print(f"ERROR: Cannot install '/etc/systemd/system/{service_name}.service'.")
                print(f"Maybe run this command with 'sudo'.")


class PID:
    def __init__(self, kp: float, ki: float, kd: float, gain: float, dead_band: float) -> None:
        self.set_point = 0.0
        self.p = 0.0
        self.i = 0.0
        self.d = 0.0
        self.kp = kp
        self.ki = ki
        self.kd = kd
        self.kg = gain
        self.dead_band = dead_band
        self.last_error = 0.0
        self.last_time = 0.0
        self.last_output = 0.0
        self.last_delta = 0.0
        self.first = True

    def process(self, error: float) -> float:
        now = time.time()

        if abs(error) <= self.dead_band:
            error = 0.0

        if self.first:
            self.first = False
            self.last_error = error
            self.last_time = now
            # return 0

        delta_time = now - self.last_time

        self.p = error
        if (self.last_error < 0 < error) or (self.last_error > 0 > error):
            self.i = 0.0
        # elif abs(error) <= 0.1:
        #     self.i = 0.0
        else:
            self.i += error * delta_time

        if delta_time > 0:
            self.d = (error - self.last_error) / delta_time

        output = self.p * self.kp + self.i * self.ki + self.d * self.kd

        output *= self.kg

        self.last_output = output
        self.last_error = error
        self.last_time = now
        self.last_delta = delta_time

        return output

    def reset(self) -> None:
        self.i = 0.0
        self.first = True

    def __str__(self) -> str:
        return f"p={self.p * self.kp:.3f}, i={self.i * self.ki:.3f}, d={self.d * self.kd:.3f}, e={self.last_error:.3f}, o={self.last_output:.3f}, ld={self.last_delta:.3f}"

    def __repr__(self) -> str:
        return f"PID({self.__str__()})"


class PlasmaPixels:
    PIXELS_PER_LIGHT = 4
    DEFAULT_BRIGHTNESS = 3
    MAX_BRIGHTNESS = 255

    def __init__(self, dat: int, clk: int) -> None:
        self.dat = dat
        self.clk = clk
        self.pixels = [[0, 0, 0, PlasmaPixels.DEFAULT_BRIGHTNESS]] * PlasmaPixels.PIXELS_PER_LIGHT

        GPIO.setup(self.dat, GPIO.OUT)
        GPIO.setup(self.clk, GPIO.OUT)

    def set_pixel(self, x: int, r: int, g: int, b: int, brightness: float = None) -> None:
        """Set the RGB value, and optionally brightness, of a single pixel.
        If you don"t supply a brightness value, the last value will be kept.
        :param x: The horizontal position of the pixel: 0 to 7
        :param r: Amount of red: 0 to 255
        :param g: Amount of green: 0 to 255
        :param b: Amount of blue: 0 to 255
        :param brightness: Brightness: 0.0 to 1.0 (default around 0.2)
        """
        if brightness is None:
            brightness = self.pixels[x][3]
        else:
            brightness = int(float(PlasmaPixels.MAX_BRIGHTNESS) * brightness) & 0b11111

        self.pixels[x] = [int(r) & 0xff, int(g) & 0xff, int(b) & 0xff, brightness]

    def set_light(self, r: int, g: int, b: int) -> None:
        """Set the RGB colour of an individual light in your Plasma chain.
        This will set all four LEDs on the Plasma light to the same colour.
        :param r: Amount of red: 0 to 255
        :param g: Amount of green: 0 to 255
        :param b: Amount of blue: 0 to 255
        """
        for x in range(4):
            self.set_pixel(x, r, g, b)

        """Output the buffer """
        self._sof()

        for pixel in self.pixels:
            r, g, b, brightness = pixel
            self._write_byte(0b11100000 | brightness)
            self._write_byte(b)
            self._write_byte(g)
            self._write_byte(r)

        self._eof()

    # Emit exactly enough clock pulses to latch the small dark die APA102s which are weird
    # for some reason it takes 36 clocks, the other IC takes just 4 (number of pixels/2)
    def _eof(self) -> None:
        GPIO.output(self.dat, 0)
        for x in range(36):
            GPIO.output(self.clk, 1)
            time.sleep(0.0000005)
            GPIO.output(self.clk, 0)
            time.sleep(0.0000005)

    def _sof(self) -> None:
        GPIO.output(self.dat, 0)
        for x in range(32):
            GPIO.output(self.clk, 1)
            time.sleep(0.0000005)
            GPIO.output(self.clk, 0)
            time.sleep(0.0000005)

    def _write_byte(self, byte: int) -> None:
        for x in range(8):
            GPIO.output(self.dat, byte & 0b10000000)
            GPIO.output(self.clk, 1)
            time.sleep(0.0000005)
            byte <<= 1
            GPIO.output(self.clk, 0)
            time.sleep(0.0000005)


class Hardware:
    def __init__(self) -> None:
        pass

    @staticmethod
    def get_cpu_temp() -> float:
        return float(subprocess.check_output(["vcgencmd", "measure_temp"])[5:-3])

    @staticmethod
    def get_cpu_freq() -> float:
        return float(subprocess.check_output(["vcgencmd", "measure_clock", "arm"])[14:-1]) / 1000000


class Fanshim:
    def __init__(self,
                 hardware: Hardware,
                 plasma_pixels: Optional[PlasmaPixels],
                 on_threshold: float,
                 off_threshold: float,
                 brightness: int,
                 fan_pin: int,
                 delay: float,
                 pwm_max_speed: int = -1,
                 pwm_min_speed: int = -1,
                 pwm_freq: int = 4,
                 pid_kp: float = 0.0,
                 pid_ki: float = 0.0,
                 pid_gain: float = 0.0) -> None:

        self.hardware = hardware

        self.run = True

        self.on_threshold = on_threshold
        self.off_threshold = off_threshold
        self.brightness = brightness
        self.delay = delay

        self.pwm_freq = pwm_freq
        self.pwm_speed = pwm_max_speed
        self.max_pwm_speed = pwm_max_speed
        self.min_pwm_speed = pwm_min_speed
        self.pid: Optional[PID] = None
        if pid_gain > 0.0:
            self.pid = PID(pid_kp, pid_ki, 0.0, pid_gain, dead_band=0.0)

        self.fan_pin = fan_pin
        self.fan_enabled = False

        self.cpu_temp = 0.0
        self.cpu_freq = 0.0

        # For FAN
        GPIO.setup(FAN_PIN, GPIO.OUT)
        GPIO.output(FAN_PIN, False)
        if self.pwm_speed > 0 or self.pid:
            self.fan_pwm = GPIO.PWM(self.fan_pin, 1)
            self.fan_pwm.start(0)
            self.fan_pwm.ChangeFrequency(self.pwm_freq)
            self.fan_pwm.ChangeDutyCycle(0)
        else:
            self.fan_pwm = None

        self.plasma_pixels = plasma_pixels

        atexit.register(self._exit)
        if self.plasma_pixels:
            self.plasma_pixels.set_light(0, 0, 0)

    def _exit(self) -> None:
        if self.plasma_pixels:
            self.plasma_pixels.set_light(0, 0, 0)
        GPIO.cleanup()

    def set_fan(self, status: bool) -> bool:
        changed = False
        if status != self.fan_enabled:
            changed = True

        if not status:
            if self.fan_pwm:
                self.fan_pwm.ChangeDutyCycle(0)
            else:
                GPIO.output(self.fan_pin, status)
        else:
            if self.fan_pwm:
                self.fan_pwm.ChangeDutyCycle(self.pwm_speed)
            else:
                GPIO.output(self.fan_pin, status)

        self.fan_enabled = status
        return changed

    def watch_temp(self) -> None:
        if level == logging.DEBUG:
            if self.pid:
                logger.debug(f"Fan Status: {self.fan_enabled}/{self.pwm_speed if self.fan_enabled else 0:.2f}% temp: {self.cpu_temp}ºC Freq {self.cpu_freq}, PID {self.pid}")
            else:
                logger.debug(f"Fan Status: {self.fan_enabled}/{self.pwm_speed if self.fan_enabled else 0:.2f}% temp: {self.cpu_temp}ºC Freq {self.cpu_freq}")

        if self.pid:
            if self.cpu_temp <= self.off_threshold:
                # self.set_fan(False)
                self.pwm_speed = 0
                self.pid.reset()

            elif self.cpu_temp > self.off_threshold:
                error = (self.cpu_temp - self.off_threshold) / (self.on_threshold - self.off_threshold)
                if error > 1.0:
                    error = 1.0
                output = self.pid.process(error)
                pwm_speed = output * self.max_pwm_speed
                if pwm_speed > self.max_pwm_speed:
                    pwm_speed = self.max_pwm_speed
                if pwm_speed < self.min_pwm_speed:
                    pwm_speed = self.min_pwm_speed
                self.pwm_speed = pwm_speed
            self.set_fan(True)

        else:
            if not self.fan_enabled and self.cpu_temp >= self.on_threshold:
                logger.info(f"{self.cpu_temp}ºC Enabling fan!")
                self.set_fan(True)
            if self.fan_enabled and self.cpu_temp <= self.off_threshold:
                logger.info(f"{self.cpu_temp}ºC Disabling fan!")
                self.set_fan(False)

        return

    def update_led_temperature(self) -> None:
        if self.plasma_pixels:
            temp = self.cpu_temp
            temp -= self.off_threshold
            temp /= float(self.on_threshold - self.off_threshold)
            temp = max(0.0, min(1.0, temp))
            temp = 1.0 - temp
            temp *= 120.0
            temp /= 360.0
            r, g, b = [int(c * 255.0) for c in colorsys.hsv_to_rgb(temp, 1.0, self.brightness / 255.0)]
            self.plasma_pixels.set_light(r, g, b)

    def main_loop(self) -> None:
        while self.run:
            time.sleep(self.delay)
            self.cpu_freq = self.hardware.get_cpu_freq()
            self.cpu_temp = self.hardware.get_cpu_temp()
            self.watch_temp()
            self.update_led_temperature()


pwm_speed = int(args.pwm_speed)
if args.pwm:
    pwm_speed = DEFAULT_PWM_SPEED
elif args.pid:
    pwm_speed = args.pid_max_pwm


plasma_pixels: Optional[PlasmaPixels] = None

if not args.no_led:
    plasma_pixels = PlasmaPixels(dat=args.led_data_pin, clk=args.led_clk_pin)

fanshim = Fanshim(
    Hardware(),
    PlasmaPixels(dat=LED_DAT_PIN, clk=LED_CLK_PIN),
    on_threshold=args.on_threshold,
    off_threshold=args.off_threshold,
    brightness=args.brightness,
    fan_pin=args.fan_pin,
    delay=args.delay,
    pwm_max_speed=pwm_speed,
    pwm_min_speed=args.pid_min_pwm,
    pid_kp=args.pid_kp if args.pid else 0.0,
    pid_ki=args.pid_ki if args.pid else 0.0,
    pid_gain=args.pid_gain if args.pid else 0.0)

if __name__ == '__main__':
    logger.info("Starting Fanshim monitor...")

    if args.pid:
        logger.info(f"  FAN is driven using PID (PI really)")
    elif pwm_speed > 0:
        logger.info(f"  FAN is driven using PWM at {pwm_speed}%")
    else:
        logger.info(f"  FAN is driven using on/off")

    logger.info(f"  Debug level {logging.getLevelName(level)}")
    logger.info(f"  Threshold: On: {args.on_threshold} Off: {args.off_threshold}")
    logger.info(f"  Delay: {args.delay}")
    if args.no_led:
        logger.info(f"  LED control: OFF")
    else:
        logger.info(f"  LED brightness: {args.brightness}")

    try:
        logger.info("Started Fanshim monitor.")
        fanshim.main_loop()
    except KeyboardInterrupt:
        logger.info("Fanshim stopped.")
