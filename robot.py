import RPi.GPIO as GPIO
from motors import *
from buttons import *
from threads import *
from talk2DE0 import *


######### LIDAR

class Lidar(object):
	def __init__(self):
		self.lidar = RPLidar('/dev/ttyUSB0')
		self.data = None
		self.flag = 0 #semaphore for multithread shared data
		#need a thread
		lidar.start_motor()
		info = lidar.get_info()
		print(info)

	def stop(self):
		self.lidar.stop()
		self.lidar.stop_motor()
		self.lidar.disconnect()
		#stop the thread

	def getData(self):
		while(self.flag):
			continue
		self.flag = 1
		data = self.data
		self.flag = 0
		return data

######### ROBOT

"""
Class with all 'physical' parts of the robot
"""
class Robot(object):
    def __init__(self):
        self.ON = 0
        self.motors = Motors() #motor 1 et motor2
		self.lidar = Lidar()
		self.buttons = Buttons()
		self.talk2DE0 = Talk2DE0()

	def encoder(self):
		v_l = self.talk2DE0.mes_left(Encoder=1)
		v_r = self.talk2DE0.mes_right(Encoder=1)
		return v_l,v_r

	def odometer(self):
		v_l = self.talk2DE0.mes_left(Encoder=0)
		v_r = self.talk2DE0.mes_right(Encoder=0)
		return v_l,v_r