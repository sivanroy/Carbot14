from simple_pid import PID
wheelP = 1
wheelI = 0
wheelD = 0

class wheelControler(object):
	def __init__(self,setpoint=0,sample=None,max=None,min=None):
		self.P = wheelP
		self.I = wheelI
		self.D = wheelD
		self.sample = sample
		self.max = max
		self.min = min
		self.pid = PID(self.P,self.I,self.D,setpoint)


"""
output = pid(v)
v=controlled_system.update(control)
"""