# Makefile for Writing Make Files Example
 
# *****************************************************
# Variables to control Makefile operation
 
CC = g++
CFLAGS = -Wall -g
LIB = -lwiringPi #-lpthread
 
# ****************************************************
# Run motor_test
 
can: main.o CAN.o SPI_CAN.o SPI.o
	$(CC) $(CFLAGS) -o can main.o CAN.o SPI_CAN.o SPI.o $(LIB)

main.o: motor_test.cc CAN.hh SPI_CAN.hh SPI.hh
	$(CC) $(CFLAGS) -c main.cpp $(LIB)


# ****************************************************

CAN.o : CAN.hh
SPI_CAN.o: SPI_CAN.hh
SPI.o: SPI.hh

clean:
	rm -rf *.o can

