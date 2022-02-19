from controlledWheels import *
#from DE02Rpi import *
from FPGA_Rpi import *

cw = ControlledWheels()
cw.start()
#dr = DE02Rpi()
speeds = [[0,0.2,0.2],[0.2,0,0.2]]
dt = 1e-1


for i in range (len(speeds[0])):
	sl = speeds[0][i]
	sr = speeds[1][i]
	cw.setSpeed(sl,sr)
	for j in range(100): #stay 1s on each value
		for i in range(int(dt/deltat)):
			#l = dr.measure(1,1)
			#r = dr.measure(1,0)
			A1,B1,A2,B2 = countPi()
			l = (A1 + B1) * 2
			r = (A2 + B2) * 2
			sl = l*wheelDia/2*RadPerTickEnc/deltat #check ??
			sr = -r*wheelDia/2*RadPerTickEnc/deltat
			if(abs(sl)>5):
				sl = 0
			if(abs(sr)>5) :
				sr = 0
			cw.sendV(sr,sl,0)
			time.sleep(deltat*.99)

cw.stop()
			