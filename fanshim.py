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
LED_DATA_PIN = 15
LED_CLK_PIN = 14
DEFAULT_PID_KP = 0.95
DEFAULT_PID_KI = 0.05
DEFAULT_PID_GAIN = 1.00
DEFAULT_PWM_SPEED = 80
DEFAULT_PID_MIN_PWM_SPEED = 40
DEFAULT_PID_MAX_PWM_SPEED = 100

DEFAULT_RAM_AMBER = 70
DEFAULT_RAM_RED = 95

DEFAULT_CPU_AMBER = 70
DEFAULT_CPU_RED = 95

logger = logging.getLogger()

parser = argparse.ArgumentParser()
parser.add_argument("-q", "--quiet", action="store_true", default=False, help="Set debugging level to quiet")
parser.add_argument("-v", "--verbose", action="store_true", default=False, help="Set debugging level to verbose")

parser.add_argument("--off-threshold", type=float, default=55.0, help="Temperature threshold in degrees C to enable fan")
parser.add_argument("--on-threshold", type=float, default=65.0, help="Temperature threshold in degrees C to disable fan")
parser.add_argument("--delay", type=float, default=2.0, help="Delay, in seconds, between temperature readings")
parser.add_argument("--no-led", action="store_true", default=False, help="Disable LED control")
parser.add_argument("--brightness", type=float, default=31.0, help="LED brightness, from 0 to 255")

parser.add_argument("--ram", action="store_true", default=False, help="Should percentage of RAM usage be used to colour LED, too")
parser.add_argument("--ram-amber", type=float, default=DEFAULT_RAM_AMBER, help=f"Percentage when RAM usage will start colouring LED amber. Default {DEFAULT_RAM_AMBER}.")
parser.add_argument("--ram-red", type=float, default=DEFAULT_RAM_RED, help=f"Percentage when RAM usage will chagne LED to red. Default {DEFAULT_RAM_RED}")
parser.add_argument("--cpu", action="store_true", default=False, help="Should percentage of CPU usage be used to colour LED, too")
parser.add_argument("--cpu-amber", type=float, default=DEFAULT_CPU_AMBER, help=f"Percentage when CPU usage will start colouring LED amber. Default {DEFAULT_CPU_AMBER}")
parser.add_argument("--cpu-red", type=float, default=DEFAULT_CPU_RED, help=f"Percentage when CPU usage will chagne LED to red. Default is {DEFAULT_CPU_RED}")

pwn_or_not = parser.add_mutually_exclusive_group()
pwn_or_not.add_argument("--pwm", action="store_true", help=f"If selected fan is going to be driven with PWM at {DEFAULT_PWM_SPEED}% of speed")
pwn_or_not.add_argument("--pwm-speed", type=int, default=-1, help="If selected fan is going to be driven with PWM at given speed betwen 0-100")
pwn_or_not.add_argument("--pid", action="store_true", help="If selected fan will be driven by PID algorithm")
pwn_or_not.add_argument("--on-off", action="store_true", help="Default if nothing else selected. No need to specify.")

parser.add_argument("--pid-kp", type=float, default=DEFAULT_PID_KP, help=f"PID proportional parameter. Used only if --pid parameter is set. Default {DEFAULT_PID_KP}")
parser.add_argument("--pid-ki", type=float, default=DEFAULT_PID_KI, help=f"PID integral parameter. Used only if --pid parameter is set. Default {DEFAULT_PID_KI}")
parser.add_argument("--pid-gain", type=float, default=DEFAULT_PID_GAIN, help=f"PID integral parameter. Used only if --pid parameter is set. Default {DEFAULT_PID_GAIN}")
parser.add_argument("--pid-min-pwm", type=float, default=DEFAULT_PID_MIN_PWM_SPEED, help=f"PWM min speed when using PID. Used only if --pid parameter is set. Default {DEFAULT_PID_MIN_PWM_SPEED}")
parser.add_argument("--pid-max-pwm", type=float, default=DEFAULT_PID_MAX_PWM_SPEED, help=f"PWM max speed when using PID. Used only if --pid parameter is set. Default {DEFAULT_PID_MAX_PWM_SPEED}")

parser.add_argument("--fan-pin", type=int, default=FAN_PIN, help=f"Fan pin, default {FAN_PIN}")
parser.add_argument("--led-clk-pin", type=int, default=LED_CLK_PIN, help=f"LED CLK pin, default {LED_CLK_PIN}")
parser.add_argument("--led-data-pin", type=int, default=LED_DATA_PIN, help=f"LED DATA pin, default {LED_DATA_PIN}")


parser.add_argument("--install", action="store_true", help="If selected only install will happen. Installing means creating file in /etc/systemd/system.")
parser.add_argument("--force", action="store_true", help="Only used if --install selected. It will allow overwriting existing file in /etc/systemd/system.")


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


def _create_command_args(args: argparse.Namespace) -> str:
    args_list = []
    if args.verbose:
        args_list.append("--verbose")
    if args.quiet:
        args_list.append("--quiet")

    args_list.append(f"--off-threshold {args.off_threshold:.1f}")
    args_list.append(f"--on-threshold {args.on_threshold:.1f}")
    args_list.append(f"--delay {args.delay}")
    if args.no_led:
        args_list.append(f"--no-led")

    args_list.append(f"--brightness {args.brightness:.1f}")

    if args.ram:
        args_list.append(f"--ram")
        args_list.append(f"--ram-amber {args.ram_amber:.1f}")
        args_list.append(f"--ram-red {args.ram_red:.1f}")

    if args.cpu:
        args_list.append(f"--cpu")
        args_list.append(f"--cpu-amber {args.cpu_amber:.1f}")
        args_list.append(f"--cpu-red {args.cpu_red:.1f}")

    if args.pwm:
        args_list.append(f"--pwm")

    if args.pwm_speed >= 0:
        args_list.append(f"--pwm-speed {args.pwm_speed}")

    if args.on_off:
        args_list.append(f"--on-off")

    if args.pid:
        args_list.append(f"--pid")
        args_list.append(f"--pid-kp {args.pid_kp}")
        args_list.append(f"--pid-ki {args.pid_ki}")
        args_list.append(f"--pid-gain {args.pid_gain}")
        args_list.append(f"--pid-min-pwm {args.pid_min_pwm}")
        args_list.append(f"--pid-max-pwm {args.pid_max_pwm}")

    if args.fan_pin != FAN_PIN:
        args_list.append(f"--fan-pin {args.fan_pin}")

    if args.led_clk_pin != LED_CLK_PIN:
        args_list.append(f"--led-clk-pin {args.led_clk_pin}")

    if args.led_data_pin != LED_DATA_PIN:
        args_list.append(f"--led-data-pin {args.led_data_pin}")

    return " ".join(args_list)


def install(args: argparse.Namespace) -> None:
    force = args.force

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
            logger.info(f"Installing /etc/systemd/system/{service_name}.service")
            try:
                with open(f"/etc/systemd/system/{service_name}.service", "wb") as f:
                    f.write(f"""
[Unit]
Description=Fanshim Service
After=rsyslog.service

[Service]
WorkingDirectory={parent_path}
ExecStart=/usr/bin/python3 {this_file} {_create_command_args(args)}
Restart=on-failure
StandardOutput=syslog
StandardError=syslog

[Install]
WantedBy=multi-user.target
""".encode("UTF-8"))

                logger.info(f"Running: systemctl enable {service_name}.service")
                os.system(f"systemctl enable {service_name}.service")

                logger.info("Daemon successfully installed. Do following to run it:")
                logger.info(f"sudo service {service_name} start")
            except PermissionError:
                logger.error(f"ERROR: Cannot install '/etc/systemd/system/{service_name}.service'.")
                logger.error(f"Maybe run this command with 'sudo'.")


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


class Bunch(dict): __getattr__ = dict.__getitem__


class Hardware:
    def __init__(self, plasma_pixels: Optional[PlasmaPixels] = None) -> None:
        self.plasma_pixels = plasma_pixels

    @staticmethod
    def get_cpu_temp() -> float:
        return float(subprocess.check_output(["vcgencmd", "measure_temp"])[5:-3])

    @staticmethod
    def get_cpu_freq() -> float:
        return float(subprocess.check_output(["vcgencmd", "measure_clock", "arm"])[14:-1]) / 1000000

    @staticmethod
    def get_ram_usage() -> Bunch:
        with os.popen('free') as f:
            _ = f.readline()
            split_line = f.readline().split()
            return Bunch(total=int(split_line[1]), used=int(split_line[2]), free=int(split_line[3]))

    @staticmethod
    def get_cpu_usage() -> Bunch:
        with os.popen("top -b -n3") as f:
            line_no = 0
            line = f.readline()
            while not line.startswith("%Cpu(s)") and line_no < 3000:
                line_no += 1
                line = f.readline()
                # print(f"1 l={line}")

            line = f.readline()
            while not line.startswith("%Cpu(s)") and line_no < 3000:
                line_no += 1
                line = f.readline()
                # print(f"2 l={line}")

            if line_no < 3000:
                split_line = line.split()
                user = float(split_line[1])
                system = float(split_line[3])
                return Bunch(total=user + system, user=user, system=system)

            return Bunch(total=line_no, user=0, system=99)


class Fanshim:
    def __init__(self,
                 hardware: Hardware,
                 on_threshold: float,
                 off_threshold: float,
                 brightness: int,
                 fan_pin: int,
                 delay: float,
                 no_led: bool,
                 pwm_max_speed: int = -1,
                 pwm_min_speed: int = -1,
                 pwm_freq: int = 4,
                 pid_kp: float = 0.0,
                 pid_ki: float = 0.0,
                 pid_gain: float = 0.0,
                 ram_amber: float = 0.0,
                 ram_red: float = 0.0,
                 cpu_amber: float = 0.0,
                 cpu_red: float = 0.0) -> None:

        self.hardware = hardware

        self.run = True

        self.on_threshold = on_threshold
        self.off_threshold = off_threshold
        self.brightness = brightness
        self.delay = delay
        self.no_led = no_led

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
        self.cpu_usage: Optional[Bunch] = None
        self.ram_usage: Optional[Bunch] = None

        self.cpu_percentage = 0.0
        self.ram_percentage = 0.0
        self.temp_percentage = 0.0
        self.light_percentage = 0.0

        self.ram_amber = ram_amber
        self.ram_red = ram_red
        self.cpu_amber = cpu_amber
        self.cpu_red = cpu_red

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

        atexit.register(self._exit)
        if self.hardware.plasma_pixels:
            self.hardware.plasma_pixels.set_light(0, 0, 0)

    def _exit(self) -> None:
        if self.hardware.plasma_pixels:
            self.hardware.plasma_pixels.set_light(0, 0, 0)
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

            debug_s = f"Fan Status: {self.fan_enabled}/{self.pwm_speed if self.fan_enabled else 0:.2f}% temp: {self.cpu_temp}ºC Freq {self.cpu_freq:.1f}"

            if self.cpu_usage:
                debug_s += f" CPU: {self.cpu_usage.total:.1f}%"
                debug_s += f" CPU LED: {self.cpu_percentage * 100.0:.1f}%"
            if self.ram_usage:
                debug_s += f" RAM: {self.ram_usage.used * 100.0 / self.ram_usage.total:.1f}%"
                debug_s += f" RAM LED: {self.ram_percentage * 100.0:.1f}%"

            debug_s += f" temp: {self.temp_percentage * 100.0:.1f}%"
            debug_s += f" light: {self.light_percentage * 100.0:.1f}%"

            if self.pid:
                debug_s += f"  PID {self.pid}"

            logger.debug(debug_s)

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

    def update_led(self) -> None:
        temp = 0
        if self.hardware.plasma_pixels:
            temp = self.cpu_temp
            temp -= self.off_threshold
            temp /= float(self.on_threshold - self.off_threshold)
            temp = max(0.0, min(1.0, temp))
            self.temp_percentage = temp

        ram = 0
        if self.ram_usage:
            ram = float(self.ram_usage.used) / float(self.ram_usage.total)
            ram -= self.ram_amber
            ram /= float(self.ram_red - self.ram_amber)
            ram = max(0.0, min(1.0, ram))
            self.ram_percentage = ram

        cpu = 0
        if self.cpu_usage:
            cpu = self.cpu_usage.total / 100.0
            cpu -= self.cpu_amber
            cpu /= float(self.cpu_red) - float(self.cpu_amber)
            cpu = max(0.0, min(1.0, cpu))
            self.cpu_percentage = cpu

        light = max(temp, ram, cpu)
        self.light_percentage = light

        light = 1.0 - light
        light *= 120.0
        light /= 360.0

        r, g, b = [int(c * 255.0) for c in colorsys.hsv_to_rgb(light, 1.0, self.brightness / 255.0)]
        self.hardware.plasma_pixels.set_light(r, g, b)

    def main_loop(self) -> None:
        while self.run:
            if self.cpu_red != 0.0:
                self.cpu_usage = self.hardware.get_cpu_usage()

            if self.ram_red != 0.0:
                self.ram_usage = self.hardware.get_ram_usage()

            self.cpu_freq = self.hardware.get_cpu_freq()
            self.cpu_temp = self.hardware.get_cpu_temp()

            self.watch_temp()

            if not self.no_led:
                self.update_led()

            time.sleep(self.delay)


pwm_speed = int(args.pwm_speed)
if args.pwm:
    pwm_speed = DEFAULT_PWM_SPEED
elif args.pid:
    pwm_speed = args.pid_max_pwm


fanshim = Fanshim(
    Hardware(plasma_pixels=PlasmaPixels(dat=args.led_data_pin, clk=args.led_clk_pin) if not args.no_led else None),
    on_threshold=args.on_threshold,
    off_threshold=args.off_threshold,
    brightness=args.brightness,
    fan_pin=args.fan_pin,
    delay=args.delay,
    no_led=args.no_led,
    pwm_max_speed=pwm_speed,
    pwm_min_speed=args.pid_min_pwm,
    pid_kp=args.pid_kp if args.pid else 0.0,
    pid_ki=args.pid_ki if args.pid else 0.0,
    pid_gain=args.pid_gain if args.pid else 0.0,
    ram_amber=(args.ram_amber / 100.0) if args.ram else 0.0,
    ram_red=(args.ram_red / 100.0) if args.ram else 0.0,
    cpu_amber=(args.cpu_amber / 100.0) if args.cpu else 0.0,
    cpu_red=(args.cpu_red / 100.0) if args.cpu else 0.0)

if __name__ == '__main__':

    if args.install:
        install(args)
    else:
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
            logger.info(f"  LED brightness: {args.brightness} (0-255)")
        if args.ram:
            logger.info(f"  RAM LED control: {fanshim.ram_amber * 100:.1f}% - {fanshim.ram_red * 100:.1f}%")
        else:
            logger.info(f"  RAM LED control: OFF")
        if args.cpu:
            logger.info(f"  CPU LED control: {fanshim.cpu_amber * 100:.1f}% - {fanshim.cpu_red * 100:.1f}%")
        else:
            logger.info(f"  CPU LED control: OFF")

        logger.info(f"  Current temperature: {fanshim.hardware.get_cpu_temp()}ºC")

        try:
            logger.info("Started Fanshim monitor.")
            fanshim.main_loop()
        except KeyboardInterrupt:
            logger.info("Fanshim stopped.")
