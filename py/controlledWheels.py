from motors import *
from DE02Rpi import *
from simple_pid import PID
import matplotlib.pyplot as plt

wheelP = 70
wheelI = 50
wheelD = 0
wheelMax = 50
wheelMin = -wheelMax
odoD = 46e-3
wheelD = 56e-3
RadPerTickEnc = (ticksEncoder/ (2*3.1415))**-1

class ControlledWheels(object):
	"""
	PID controlled wheels motors
	use:setSpeed(sl,sr)
		start() / stop()
		sendV(current_s_l,current_s_r)
	"""
	def __init__(self):
		self.motors = Motors()
		self.leftPID = PID(wheelP,wheelI,wheelD,setpoint = 0)
		self.rightPID = PID(wheelP,wheelI,wheelD, setpoint = 0)
		self.io_init()

	def io_init(self):
		self.leftPID.sample_time = deltat
		self.rightPID.sample_time= deltat
		self.leftPID.output_limits = (wheelMin,wheelMax)
		self.rightPID.output_limits = (wheelMin,wheelMax)
		self.motors.start_all()

	def setSpeed(self,sl,sr):
		self.leftPID.setpoint = sl
		self.rightPID.setpoint = sr

	def stop(self):
		self.setSpeed(0,0)
		self.leftPID.set_auto_mode = False
		self.rightPID.set_auto_mode = False
		self.motors.stop_all()

	def start(self):
		self.leftPID.set_auto_mode = True
		self.rightPID.set_auto_mode = True
		self.motors.start_all()

	def giveV(self,current_s_l,current_s_r):
		"""
		give output of PID controller
		! current value must be in m/s
		"""
		out_l = self.leftPID(current_s_l)
		out_r = self.rightPID(current_s_r)
		return out_l,out_r

	def sendV(self,current_s_l,current_s_r,verbose=0):
		v_l,v_r = self.giveV(current_s_l,current_s_r)
		v_l = int(v_l)
		v_r = int(v_r)
		self.motors.set_speeds(v_l,v_r)
		if(verbose):print("v_l = {}, v_r = {}".format(v_l,v_r))



values = [0,20e-2,50e-2]#,1e-2,-1e-2,0]

### displacement tests
cw = ControlledWheels()
cw.start()
dr = DE02Rpi()
dt = 1e-1
cw.leftPID.sample_time = dt
cw.rightPID.sample_time = dt
sl_m = []
sl_r = []
setp = []
for si in values:
	cw.setSpeed(si,si)
	for j in range(100): #stay 1s on each value
		for i in range(int(dt/deltat)):
			l = dr.measure(1,1)
			r = dr.measure(1,0)
			sl = -l*wheelD/2*RadPerTickEnc/deltat #check ??
			sr = r*wheelD/2*RadPerTickEnc/deltat
			sl_m.append(sl)
			sl_r.append(sr)
			setp.append(si)
			print("set speed : {}".format(si))
			print("sl = {}; sr = {}".format(sl,sr))
			cw.sendV(sl,sr,1)
			time.sleep(deltat)

cw.stop()
			
plt.plot(sl_m)
plt.plot(sl_r)
plt.plot(setp)
plt.show()
