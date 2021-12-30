import RPi.GPIO as GPIO
from motors import *
from buttons import *
from threads import *
from DE02Rpi import *

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
		self.DE02Rpi = DE02Rpi()

	#manière de rendre physique les parties nécessitant SPI
	def encoder(self):
		v_l = self.DE02Rpi.mes_left(Encoder=1)
		v_r = self.DE02Rpi.mes_right(Encoder=1)
		return v_l,v_r

	def odometer(self):
		v_l = self.DE02Rpi.mes_left(Encoder=0)
		v_r = self.DE02Rpi.mes_right(Encoder=0)
		return v_l,v_r

	#functions allumer et eteindre le robot
	def isON(self):
        return self.ON

    def activate(self):
        print("Activate the robot \n")
        self.ON = 1

    def shutdown(self):
        print("Shutdown the Robot \n")
        self.lidar.stop()
        #stop threads (lidar and other if)
        self.motors.set_speed(0,0)
        self.ON = 0