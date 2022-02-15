# Makefile for Writing Make Files Example
 
# *****************************************************
# Variables to control Makefile operation
 
CC = g++
CFLAGS = -Wall -g
LIB = -lwiringPi -lpthread
 
# ****************************************************
# Targets needed to bring the executable up to date
 
mtest: motor_test.o DualMC33926RPi.o
	$(CC) $(CFLAGS) -o mtest motor_test.o DualMC33926RPi.o $(LIB)

# The motor_test.o target can be written more simply
 
motor_test.o: motor_test.cpp DualMC33926RPi.h
	$(CC) $(CFLAGS) -c motor_test.cpp $(LIB)

DualMC33926RPi.o: DualMC33926RPi.h


enctest: DE02Rpi_test.o DE02Rpi.o
	$(CC) $(CFLAGS) -o enctest DE02Rpi_test.o DE02Rpi.o $(LIB)

DE02Rpi_test.o: DE02Rpi_test.cpp DE02Rpi.h
	$(CC) $(CFLAGS) -c DE02Rpi_test.cpp $(LIB)

DE02Rpi.o: DE02Rpi.h

# PID

#pid.o: PidLib/pid.h
