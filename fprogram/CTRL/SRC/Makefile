CC = g++
CFLAGS = -Wall -g
LIB = -lwiringPi

CVS = ../ctrlStruct
IO = ../ctrl_IO
LLC = ../lowLevelCtrl
MP = ../myPosition
MLCPF = ../midLevelCtrlPF
CAN = ../LIB/CAN
D2R = ../LIB/DE02Rpi


CTRL_O = $(CVS)/ctrlStruct.o $(IO)/ctrlIn.o $(IO)/ctrlOut.o $(LLC)/lowLevelCtrl.o $(MP)/myPosition.o \
         $(MLCPF)/midLevelCtrlPF.o $(CAN)/CAN.o $(D2R)/DE02Rpi.o

CTRL_H = $(CVS)/ctrlStruct.h $(IO)/ctrlIn.h $(IO)/ctrlOut.h $(LLC)/lowLevelCtrl.h $(MP)/myPosition.h \
         $(MLCPF)/midLevelCtrlPF.h $(CAN)/CAN.h $(D2R)/DE02Rpi.h


# ****************************************************
# Run motors_test

mtest: motors_test.o $(CTRL_O)
	$(CC) $(CFLAGS) -o mtest motors_test.o $(CTRL_O) $(LIB)

motors_test.o: motors_test.cpp $(CTRL_H)
	$(CC) $(CFLAGS) -c motors_test.cpp $(LIB)


# ****************************************************
# Run llc_test

llctest: llc_test.o $(CTRL_O)
	$(CC) $(CFLAGS) -o llctest llc_test.o $(CTRL_O) $(LIB)

llc_test.o: llc_test.cpp $(CTRL_H)
	$(CC) $(CFLAGS) -c llc_test.cpp $(LIB)


# ****************************************************
# .o: .h

$(CVS)/ctrlStruct.o: $(CVS)/ctrlStruct.h
$(IO)/ctrlIn.o: $(IO)/ctrlIn.h
$(IO)/ctrlOut.o: $(IO)/ctrlOut.h
$(LLC)/lowLevelCtrl.o: $(LLC)/lowLevelCtrl.h
$(MP)/myPosition.o: $(MP)/myPosition.h
$(MLCPF)/midLevelCtrlPF.o: $(MLCPF)/midLevelCtrlPF.h
$(CAN)/CAN.o: $(CAN)/CAN.h
$(D2R)/DE02Rpi.o: $(D2R)/DE02Rpi.h


# ****************************************************
# Clean

clean:
	rm -rf *.o mtest llctest