import spidev
import math
#import keyboard
import RPi.GPIO as GPIO
import time

ticksEncoder = 1840#2048*4
ticksOdometer = 7900 #9000 approx
deltat = 10e-3

class DE02Rpi(object):
    def __init__(self):
        self.MySPI_FPGA = spidev.SpiDev()
        self.MySPI_FPGA.open(0,0)
        self.MySPI_FPGA.max_speed_hz = 500000
        GPIO.setmode(GPIO.BCM)
        GPIO.setwarnings(False)       

    def count(self,spi,verbose=0):
        treshold = 4192
        value = spi[4] + (spi[3] << 8) + \
            (spi[2] << 16) + (spi[1] << 24) - treshold
        if (value == - treshold):
            #if (verbose) :
            print("Maybe is there an error (: Did you Program the DE0?")
        return value

    def giveAddress(self,encoder,left,sonar):
        Adr = None
        if (left == 0):
            if(encoder == 1):
                Adr = 0x00
            else :
                Adr = 0x02
        elif (left == 1):
            if(encoder == 1):
                Adr = 0x01
            else :
                Adr = 0x03

        if(sonar==0):
            Adr = 0x04
        if(sonar ==1):
            Adr = 0x05
        if(sonar == 2):
            Adr = 0x01
        return Adr

    def measure(self,encoder=1,left=1,sonar=-1,verbose=0):
        Adr = self.giveAddress(encoder,left,sonar)
        if(verbose):print("Adr = {}".format(Adr))
        ToSPI = [Adr, 0x00, 0x00, 0x00, 0x00]
        A = self.MySPI_FPGA.xfer2(ToSPI)
        if (sonar == -1) :
            count = self.count(A,verbose=verbose)
        else :
            return A
        if (verbose): print("Count = {}".format(count))
        return count


#de0 tests


de0 = DE02Rpi()
while(1):
    A = de0.measure(-1,-1,2)
    value = A[4] + (A[3] << 8) + (A[2] << 16)
    if (A[1] >= 128):
        value = value + ((A[1]-128) << 24 ) - 2**31
    else :
        value = value + (A[1] << 24)
    print(value)
    time.sleep(0.01)

"""
de0 = DE02Rpi()
count = 0
i = 0
while(1):
    for i in range(1000): #5sec
        d1 = de0.measure(sonar=0)
        d2 = de0.measure(sonar=1)
        print("d1:{} cm | d2:{}".format(d1,d2))
        time.sleep(0.01)
"""
"""
de0 = DE02Rpi()
count = 0
i = 0
while(1):
    a = int(input("give left + encoder*2 :\nwhere left and encoder is bool\n"))
    l = 0; e = 0
    if (a%2 == 1):
        l = 1
    if(a-2>=0):
        e = 1
    for i in range(1000): #5sec
        d = de0.measure(encoder=e,left=l)
        count += d
        #print(d)
        print(count)
        i += 1
        time.sleep(0.01)
"""
