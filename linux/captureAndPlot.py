# to use the serial library and without needing sudo:
# pip install pyserial
# sudo usermod -a -G dialout <username>
# need to reboot afterwards

import serial
import time
import numpy as np
import matplotlib.pyplot as plt

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

ser = serial.Serial("/dev/ttyACM0", 115200)

ser.write(b'z')
data = ser.read(1)
print(data)

ser.write(b'p00125')
data = ser.read(6)
print(data)

ser.write(b'w60000')
data = ser.read(6)
print(data)

ser.write(b'd30000')
data = ser.read(6)
print(data)

ser.write(b'c')
data = ser.read(1)
print(data)

time.sleep(1)

ser.write(b'd00000')
data = ser.read(6)
print(data)

ser.write(b'm')
data = ser.read(1)
print(data)

binData = b""
for i in range(0, 2048):
    ser.write(b'l')
    binData = binData + ser.read(64)

print(f"len(binData) = {len(binData)}")
fd = open("data1.txt", "w")
for item in binData:
    fd.write(str(item) + "\n")
fd.close()

plotData("data1.txt")

