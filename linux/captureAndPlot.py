# to use the serial library and without needing sudo:
# pip install pyserial
# sudo usermod -a -G dialout <username>
# need to reboot afterwards

import serial
import time
import sys
import numpy as np
import matplotlib.pyplot as plt

###################################################################################################

def plotData(filename):
    fd = open(filename, "r")
    adcStrList = fd.read().strip().split("\n")
    fd.close()

    VOLTS_PER_COUNT = 3.3 / (2**8 - 1) # 0-3.3V is 0-255 counts
    adcIntList = [int(s) * VOLTS_PER_COUNT for s in adcStrList]
    timeList = [i/500000.0 for i in range(0, len(adcIntList))]

    print(f"length of adcIntList = {len(adcIntList)} first = {adcIntList[0]} last = {adcIntList[len(adcIntList)-1]}")
    print(f"length of timeList = {len(timeList)}")

    y = np.array(adcIntList)
    x = np.array(timeList)

    plt.plot(x, y)
    plt.xlabel("Time [s]")
    plt.ylabel("Voltage [V]")
    plt.axis([0, x[-1], 0, 3.4])
    plt.show()

###################################################################################################

def pwmOn(ser):
    # 125 MHz / 125 = 1MHz counter
    ser.write(b"p00125")
    data = ser.read(6)
    print(data)

    # 1000 counts / 1MHz = 0.001 sec PWM period (1 kHz)
    ser.write(b"w01000")
    data = ser.read(6)
    print(data)

    # duty cycle is half of the wrap value of 1000
    ser.write(b"d00500")
    data = ser.read(6)
    print(data)

###################################################################################################

def pwmOff(ser):
    ser.write(b"d00000")
    data = ser.read(6)
    print(data)

    ser.write(b"m")
    data = ser.read(1)
    print(data)

###################################################################################################

def doCapture(ser):
    ser.write(b"c")
    data = ser.read(1)
    print(data)

    time.sleep(1)

    binData = b""
    for i in range(0, 2048):
        ser.write(b"l")
        binData = binData + ser.read(64)
        
    print(f"len(binData) = {len(binData)}")
    fd = open("data1.txt", "w")
    for item in binData:
        fd.write(str(item) + "\n")
    fd.close()

###################################################################################################

debug = False
if len(sys.argv) > 1:
    if sys.argv[1] == "debug":
        debug = True

ser = serial.Serial("/dev/ttyACM0", 115200)

if debug:
    pwmOn(ser)
doCapture(ser)
if debug:
    pwmOff(ser)
plotData("data1.txt")

