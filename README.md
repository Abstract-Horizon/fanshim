# fanshim

This is another take on python script for Pimoroni's Fanshim (https://shop.pimoroni.com/products/fan-shim)

Original repository is here: https://github.com/pimoroni/fanshim-python

Motivation behind this is to simplify Python code - to deliver it without need of extra
steps like installing new libraries, manual creation of systemd service and adding simple
PWM in order to preserve bearings on the little fan. (for instance PWM of 80% works much quieter than
when fan is full on).

This is derivative work from orginal repository, work from https://github.com/jane-t/rpi-fanshim
where LED is accessed using simple python GPIO code and PWM code from https://github.com/grayerbeard/fanshim-python-pwm

## Features

This Python code adds following features to existing fanshim code:

- running at speed smaller than 100% (`--pwm` or `--pwm-speed <0-100>` options)
- running LED at predefined brightness (`--brightness <0-255>` where default is 31)
- 'install' option to create systemd service (`--install` with `--force` if it is already installed)
- verbosity of output (`--quiet`/`-q`, nothing or `--verbose`/`-v` for warning, info and debug output levels)
- PID algorithm (PI really) with PWM. Options invoked with `--pid`. It can be fine tuned with following options
  - `--pid-kp <p>` for proportional component (0.0 - 1.0)
  - `--pid-ki <i>` for integral component (0.0 - 1.0)
  - `--pid-gain <gain>` for overall gain (float bigger than 0.0)
  - `--pid-min-pwm <speed>` for min PWM speed (if PID is to drive fan) - where speed is between 0 - 100
  - `--pid-max-pwm <speed>` for max PWM speed (if PID is to drive fan) - where speed is between 0 - 100 (this is similar to `--pwm-speed` in hysteresis driven fan)
- ability to change fan and LED pins (`--fan-pin <pin>`, `--led-clk-pin <pin>` and `--led-data-pin <pin>` with defaults of 18, 15 and 14)
- option for LED not to be driven (`--no-led`)
- option for CPU and RAM to affect LED. Whichever is bigger - temperature, CPU usage or RAM usage will be reflected on LED
  - CPU percentage is configured with `--cpu` switch with `--cpu-amber <percentage>` for when LED starts changing from green towards red and `--cpu-red <percentage>` and it is lit red only
  - RAM usage percentage is configured with `--ram` switch with `--ram-amber <percentage>` for when LED starts changing from green towards red and `--ram-red <percentage>` and it is lit red only
- option to define hysteresis with `--off-threshold <temperature>` and `--on-threshold <temperature`
- option how often temperature, CPU and RAM usage will be checked `--delay <seconds>`

## Usage

For instance:

```bash
/fanshim.py --off-threshold 60.0 --on-threshold 75.0 --delay 2.0 --brightness 31.0 --ram --ram-amber 70.0 --ram-red 90.0 --cpu --cpu-amber 70.0 --cpu-red 90.0 --pwm-speed 80
```

will turn the fan with 80% PWM when temperature reaches 75ºC and switch it off when it reaches 60ºC.
Also, temperatures between 60ºC and 75ºC will be reflected with various colours between green, amber and red.
Everything below 60ºC and LED will be green and above 75ºC will be red.

Also, LED will turn between green, amber and red if CPU usage goes over 70% up to 90% where after that it'll stay red.
Same goes for RAM usage (70% to 90%). LED will have maximum brightness of 31 out of 255.

Last thing is that temperature, CPU and RAM usage will be tested every 2 seconds.


## Installation

Download fanshim.py (or clone this repository) on Raspberry Pi.

If you have downloaded it - it might be necessary to add 'execution' flag:

```bash
chmod u+x fanshim.py
```

After that you're ready to install it:

```bash
sudo ./fanshim.py --install
```

The above will install it with default parameters (they can be seen if you execute it with `./fanshim.py -h')

When installed you can see service file with:

```bash
cat /etc/systemd/system/fanshim.service
```

To start it all you need to do is:

```bash
sudo service fanshim start
```

You can see it running with:


```bash
sudo service fanshim status
```

and logs with:
```bash
sudo journalctl -u fanshim
```
