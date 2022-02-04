import spidev
import math
#import keyboard
import RPi.GPIO as GPIO
import time

ticksEncoder = 2048*4
ticksOdometer = 7900 #9000 approx
deltat = 10e-3

class DE02Rpi(object):
    def __init__(self):
        self.MySPI_FPGA = spidev.SpiDev()
        self.MySPI_FPGA.open(0,1)
        self.MySPI_FPGA.max_speed_hz = 500000
        GPIO.setmode(GPIO.BCM)
        GPIO.setwarnings(False)       

    def count(self,spi,verbose=0):
        treshold = 4192
        value = spi[4] + (spi[3] << 8) + (spi[2] << 16) + (spi[1] << 24) - treshold
        if (value == - treshold):
            #if (verbose) :
            print("Maybe is there an error (: Did you Program the DE0?")
        return value

    def giveAddress(self,encoder,left):
        Adr = None
        if (left == 1):
            if(encoder == 1):
                Adr = 0x00
            else :
                Adr = 0x02
        elif (left == 0):
            if(encoder == 1):
                Adr = 0x01
            else :
                Adr = 0x03
        return Adr

    def measure(self,encoder=1,left=1,verbose=0):
        Adr = self.giveAddress(encoder,left)
        ToSPI = [Adr, 0x00, 0x00, 0x00, 0x00]
        A = self.MySPI_FPGA.xfer2(ToSPI)
        count = self.count(A,verbose=verbose)
        if (verbose): print(count)
        return count

de0 = DE02Rpi()
count = 0
i = 0
while(1):
    d = de0.measure(encoder=0)
    count += d
    #print(d)
    print(count)
    i += 1
    time.sleep(0.01)