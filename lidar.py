#import RPi.GPIO as GPIO
from rplidar import RPLidar
from threads import *

def lidar_thread_function(MyLidar):
    #this for acts as a while (: into the data of the lidar
    for i, lidar_scan in enumerate(MyLidar.iter_scans()):
        #set scan data
        theta = []; dist = []
        for scan in (lidar_scan):  # each scan = [quality,angle,dist]
            theta.append(scan[1])
            dist.append(scan[2])
        #semaphored
        if (MyLidar.threadOn == 0):
            break
        while(self.flag):
            continue
        self.flag = 1
        self.data = [dist,theta]
        self.flag = 0
        #semaphored
        if (MyLidar.threadOn == 0):
            break
    #ended lidar task
    MyLidar.stop()


class Lidar(object):
    def __init__(self):
        self.lidar = RPLidar('/dev/ttyUSB0')
        self.threadOn = 1 #shutdown thread
        self.data = None
        self.flag = 0 #semaphore for multithread shared data
        self.thread = myThread(lidar_thread_function,self,'lidar')

    def start(self):
        lidar.start_motor()
        info = lidar.get_info()
        print(info)
        self.thread.start()

    def stop(self):
        self.threadOn = 0
        self.lidar.stop()
        self.lidar.stop_motor()
        self.lidar.disconnect()
        #stop the thread

    #if no data get , return None !!
    def getData(self):
        while(self.flag):
            continue
        self.flag = 1
        data = self.data
        self.flag = 0
        return data