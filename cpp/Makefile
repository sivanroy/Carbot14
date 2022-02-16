# Makefile for Writing Make Files Example
 
# *****************************************************
# Variables to control Makefile operation
 
CC = g++
CFLAGS = -Wall -g
LIB = -lwiringPi -lpthread
 
# ****************************************************
# Run motor_test
 
mtest: motor_test.o DualMC33926RPi.o
	$(CC) $(CFLAGS) -o mtest motor_test.o DualMC33926RPi.o $(LIB)

motor_test.o: motor_test.cpp DualMC33926RPi.h
	$(CC) $(CFLAGS) -c motor_test.cpp $(LIB)

#DualMC33926RPi.o: DualMC33926RPi.h

# ****************************************************
# Run DE02Rpi_test

enctest: DE02Rpi_test.o DE02Rpi.o
	$(CC) $(CFLAGS) -o enctest DE02Rpi_test.o DE02Rpi.o $(LIB)

DE02Rpi_test.o: DE02Rpi_test.cpp DE02Rpi.h
	$(CC) $(CFLAGS) -c DE02Rpi_test.cpp $(LIB)

#DE02Rpi.o: DE02Rpi.h

# ****************************************************
# PID

conttest: contWheels_test.o controlledWheels.o DE02Rpi.o DualMC33926RPi.o pid.o
	$(CC) $(CFLAGS) -o conttest contWheels_test.o controlledWheels.o DE02Rpi.o DualMC33926RPi.o pid.o $(LIB)

contWheels_test.o: DE02Rpi_test.cpp controlledWheels.h DE02Rpi.h DualMC33926RPi.h pid.h
	$(CC) $(CFLAGS) -c contWheels_test.cpp $(LIB)

controlledWheels.o: controlledWheels.h
DE02Rpi.o: DE02Rpi.h
DualMC33926RPi.o: DualMC33926RPi.h
pid.o: pid.h


.PHONY: clean

clean:
	rm -rf *.o mtest enctest conttest

