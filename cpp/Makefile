# Makefile for Writing Make Files Example
 
# *****************************************************
# Variables to control Makefile operation
 
CC = g++
CFLAGS = -Wall -g
LIB = -lwiringPi -lpthread
 
# ****************************************************
# Run motor_test
 
mtest: motor_test.o CAN.o
	$(CC) $(CFLAGS) -o mtest motor_test.o CAN.o $(LIB)

motor_test.o: motor_test.cpp CAN.h
	$(CC) $(CFLAGS) -c motor_test.cpp $(LIB)


# ****************************************************
# Run DE02Rpi_test

enctest: DE02Rpi_test.o DE02Rpi.o
	$(CC) $(CFLAGS) -o enctest DE02Rpi_test.o DE02Rpi.o $(LIB)

DE02Rpi_test.o: DE02Rpi_test.cpp DE02Rpi.h
	$(CC) $(CFLAGS) -c DE02Rpi_test.cpp $(LIB)


# ****************************************************
# Run EncData_test

ticktest: EncData_test.o DE02Rpi.o CAN.o
	$(CC) $(CFLAGS) -o ticktest EncData_test.o DE02Rpi.o CAN.o $(LIB)

EncData_test.o: EncData_test.cpp DE02Rpi.h CAN.h
	$(CC) $(CFLAGS) -c EncData_test.cpp $(LIB)


# ****************************************************
# Run EncCalib

EncCalib: EncCalib.o DE02Rpi.o CAN.o
	$(CC) $(CFLAGS) -o EncCalib EncCalib.o DE02Rpi.o CAN.o $(LIB)

EncCalib.o: EncCalib.cpp DE02Rpi.h CAN.h
	$(CC) $(CFLAGS) -c EncCalib.cpp $(LIB)



# ****************************************************
# PID

conttest: contWheels_test.o controlledWheels.o DE02Rpi.o CAN.o pid.o
	$(CC) $(CFLAGS) -o conttest contWheels_test.o controlledWheels.o DE02Rpi.o CAN.o pid.o $(LIB)

contWheels_test.o: contWheels_test.cpp controlledWheels.h DE02Rpi.h CAN.h pid.h
	$(CC) $(CFLAGS) -c contWheels_test.cpp $(LIB)


cantest: can_test.o
	$(CC) $(CFLAGS) -o cantest can_test.o

can_test.o: can_test.cpp
	$(CC) $(CFLAGS) -c can_test.cpp


controlledWheels.o: controlledWheels.h
DE02Rpi.o: DE02Rpi.h
CAN.o: CAN.h
pid.o: pid.h


clean:
	rm -rf *.o mtest enctest conttest ticktest cantest EncCalib

