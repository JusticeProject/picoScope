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

plotData("data1.txt")
plotData("data2.txt")
plotData("data3.txt")
