# Makefile for Writing Make Files Example
 
# *****************************************************
# Variables to control Makefile operation
 
CC = g++
CFLAGS = -Wall -g
LIB = -lwiringPi #-lpthread


# ****************************************************
# Run motor_test

teensy: test_switch.o Lib/rs232/rs232.o
	$(CC) $(CFLAGS) -o teensy test_switch.o Lib/rs232/rs232.o $(LIB)

test_switch.o: test_switch.cpp Lib/rs232/rs232.h
	$(CC) $(CFLAGS) -c test_switch.cpp $(LIB)


# ****************************************************
# Run socketCAN_test

stest: socketCAN_test.o
	$(CC) $(CFLAGS) -o stest socketCAN_test.o $(LIB)

socketCAN_test.o: socketCAN_test.cpp
	$(CC) $(CFLAGS) -c socketCAN_test.cpp $(LIB)

 
# ****************************************************
# Run motor_test
 
mtest: motor_test.o Lib/CAN/CAN.o
	$(CC) $(CFLAGS) -o mtest motor_test.o Lib/CAN/CAN.o $(LIB)

motor_test.o: motor_test.cpp Lib/CAN/CAN.h
	$(CC) $(CFLAGS) -c motor_test.cpp $(LIB)


# ****************************************************
# Run DE02Rpi_test

enctest: DE02Rpi_test.o Lib/DE02Rpi/DE02Rpi.o
	$(CC) $(CFLAGS) -o enctest DE02Rpi_test.o Lib/DE02Rpi/DE02Rpi.o $(LIB)

DE02Rpi_test.o: DE02Rpi_test.cpp Lib/DE02Rpi/DE02Rpi.h
	$(CC) $(CFLAGS) -c DE02Rpi_test.cpp $(LIB)


# ****************************************************
# Run EncData_test

ticktest: EncData_test.o Lib/DE02Rpi/DE02Rpi.o Lib/CAN/CAN.o
	$(CC) $(CFLAGS) -o ticktest EncData_test.o Lib/DE02Rpi/DE02Rpi.o Lib/CAN/CAN.o $(LIB)

EncData_test.o: EncData_test.cpp Lib/DE02Rpi/DE02Rpi.h Lib/CAN/CAN.h
	$(CC) $(CFLAGS) -c EncData_test.cpp $(LIB)


# ****************************************************
# Run EncCalib

EncCalib: EncCalib.o Lib/DE02Rpi/DE02Rpi.o Lib/CAN/CAN.o
	$(CC) $(CFLAGS) -o EncCalib EncCalib.o Lib/DE02Rpi/DE02Rpi.o Lib/CAN/CAN.o $(LIB)

EncCalib.o: EncCalib.cpp Lib/DE02Rpi/DE02Rpi.h Lib/CAN/CAN.h
	$(CC) $(CFLAGS) -c EncCalib.cpp $(LIB)



# ****************************************************
# PID

conttest: contWheels_test.o controlledWheels.o Lib/DE02Rpi/DE02Rpi.o Lib/CAN/CAN.o Lib/PID/pid.o
	$(CC) $(CFLAGS) -o conttest contWheels_test.o controlledWheels.o Lib/DE02Rpi/DE02Rpi.o Lib/CAN/CAN.o Lib/PID/pid.o $(LIB)

contWheels_test.o: contWheels_test.cpp controlledWheels.h Lib/DE02Rpi/DE02Rpi.h Lib/CAN/CAN.h Lib/PID/pid.h
	$(CC) $(CFLAGS) -c contWheels_test.cpp $(LIB)


cantest: can_test.o
	$(CC) $(CFLAGS) -o cantest can_test.o

can_test.o: can_test.cpp
	$(CC) $(CFLAGS) -c can_test.cpp


controlledWheels.o: controlledWheels.h
Lib/DE02Rpi/DE02Rpi.o: Lib/DE02Rpi/DE02Rpi.h
Lib/CAN/CAN.o: Lib/CAN/CAN.h
Lib/PID/pid.o: Lib/PID/pid.h
Lib/rs232/rs232.o: Lib/rs232/rs232.h


clean:
	rm -rf *.o teensy stest mtest enctest conttest ticktest cantest EncCalib

