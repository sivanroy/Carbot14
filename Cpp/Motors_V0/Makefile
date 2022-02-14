# Makefile for Writing Make Files Example
 
# *****************************************************
# Variables to control Makefile operation
 
CC = g++
CFLAGS = -Wall -g
LIB = -lwiringPi
 
# ****************************************************
# Targets needed to bring the executable up to date
 
main: main.o DualMC33926RPi.o
	$(CC) $(CFLAGS) -o main main.o DualMC33926RPi.o $(LIB)
 
# The main.o target can be written more simply
 
main.o: main.cpp DualMC33926RPi.h
	$(CC) $(CFLAGS) -c main.cpp $(LIB)
 
DualMC33926RPi.o: DualMC33926RPi.h
