import numpy as np
import matplotlib.pyplot as plt

SAMPLE_FREQ = 500000 # Hz

def showFFT(filename):
    fd = open(filename, "r")
    adcStrList = fd.read().strip().split("\n")
    fd.close()

    NUM_SAMPLES = len(adcStrList)
    signal = np.array(adcStrList)

    # use the fft with real inputs
    result = np.fft.rfft(signal)
    magnitudes = np.absolute(result) * 2 / NUM_SAMPLES
    freq = np.fft.rfftfreq(NUM_SAMPLES) * SAMPLE_FREQ # get the frequency bins

    # plot the frequency domain
    plt.plot(freq, magnitudes)
    plt.show()

showFFT("data1.txt")
showFFT("data2.txt")
