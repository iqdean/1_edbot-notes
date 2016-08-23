#!/bin/sh

# Ref: 
# http://evolutek.org/howto-activate-arduino-serial-on-intel-edison.html

# export all useful gpio
echo 130 > /sys/class/gpio/export
echo 248 > /sys/class/gpio/export
echo 216 > /sys/class/gpio/export
echo 131 > /sys/class/gpio/export
echo 249 > /sys/class/gpio/export
echo 217 > /sys/class/gpio/export
echo 214 > /sys/class/gpio/export

# Disable tri-stat
echo low > /sys/class/gpio/gpio214/direction

# Set direction
echo low  > /sys/class/gpio/gpio248/direction
echo high > /sys/class/gpio/gpio249/direction

echo in > /sys/class/gpio/gpio216/direction
echo in > /sys/class/gpio/gpio217/direction

# Set Mode1
echo mode1 > /sys/kernel/debug/gpio_debug/gpio130/current_pinmux
echo mode1 > /sys/kernel/debug/gpio_debug/gpio131/current_pinmux

# Activate tri-state
echo high > /sys/class/gpio/gpio214/direction

