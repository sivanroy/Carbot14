import RPi.GPIO as GPIO
import time

deltat = 0.05

def countPi():
	count = 0
	GPIO.setwarnings(False) # Ignore warning for now
	GPIO.setmode(GPIO.BCM)
	GPIO.setup(19, GPIO.IN)
	GPIO.setup(26, GPIO.IN)
	GPIO.setup(20, GPIO.IN)
	GPIO.setup(21, GPIO.IN)
	countA1 = 0
	countA2 = 0
	countB1 = 0
	countB2 = 0
	i = 0
	while(i<20500): #100k = 0.25 sec
		i+=1
		if(GPIO.input(19)):
			countA1 += 1
		if(GPIO.input(26)):
			countB1 += 1
		if(GPIO.input(20)):
			countA2 += 1
		if(GPIO.input(21)):
			countB2 += 1
	return countA1,countA2,countB1,countB2

"""
t1 = time.time()
A1,B1,A2,B2 = countPi()
t2 = time.time()

print("{} {} {} {}".format(A1,B1,A2,B2))
print("time = {}".format(t2-t1))
"""