# https://github.com/intel-iot-devkit/mraa/tree/master/examples/python

import mraa
import os
import sys
import time

class Counter:
    count = 0

c = Counter()

def pwrBtnIsr(gpio):
    print("pin " + repr(gpio.getPin()) + " = " + repr(gpio.read()) + " Count : " + repr(c.count))
    os.system("sudo shutdown -h now")
    c.count+=1

pin = 14      # this is a global var shared with isr and main code

if (len(sys.argv) == 2):
	pin = int(sys.argv[1], 10)

try:
    print("Using pin : " + repr(pin))
    x = mraa.Gpio(pin)
    print("Starting ISR for pin " + repr(pin))
    x.dir(mraa.DIR_IN)          #  wire momentary PUSH_BTN b/t pin 14 and GND
    x.mode(mraa.MODE_PULLUP)    #< PUSH pulls low, RELEASE pulls high, get irq as expected  
    #x.mode(mraa.MODE_PULLDOWN) #< PUSH and RLS do nothing cuz pin is pulled down so no irq
    x.isr(mraa.EDGE_BOTH, pwrBtnIsr, x)
    var = raw_input("Press ENTER to stop")
    x.isrExit()
except ValueError as e:
    print(e)
