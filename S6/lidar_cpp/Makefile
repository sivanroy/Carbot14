CC = g++
CFLAGS = -Wall -g
LIB = -lwiringPi -lpthread

L1 = RPLidar/abs_rxtx.h
L2 = RPLidar/arch_linux.h
L3 = RPLidar/assert.h
L4 = RPLidar/byteops.h
L5 = RPLidar/event.h
L6 = RPLidar/locker.h
L7 = RPLidar/net_serial.h
L8 = RPLidar/rplidar.h
L9 = RPLidar/rplidar_cmd.h
L10 = RPLidar/rplidar_driver.h
L11 = RPLidar/rplidar_protocol.h
L12 = RPLidar/rptypes.h
L13 = RPLidar/sdkcommon.h
L14 = RPLidar/sl_crc.h
L15 = RPLidar/sl_lidar.h
L16 = RPLidar/sl_lidar_cmd.h
L17 = RPLidar/sl_lidar_driver.h
L18 = RPLidar/sl_lidar_driver_impl.h
L19 = RPLidar/sl_lidar_protocol.h
L20 = RPLidar/sl_types.h
L21 = RPLidar/socket.h
L22 = RPLidar/thread.h
L23 = RPLidar/timer.h
L24 = RPLidar/types.h
L25 = RPLidar/util.h

LIDAR_H = $(L1) $(L2) $(L3) $(L4) $(L5) $(L6) $(L7) $(L8) $(L9) $(L10) $(L11) $(L12) $(L13) $(L14) $(L15) $(L16) $(L17) $(L18) $(L19) $(L20) $(L21) $(L22) $(L23) $(L24) $(L25)
LIDAR_O = $(LIDAR_H:.h=.o)

lidar: lidar_test.o $(LIDAR_O)
	$(CC) $(CFLAGS) -o lidar lidar_test.o $(LIDAR_O) $(LIB)

lidar_test.o: lidar_test.cpp $(LIDAR_H)
	$(CC) $(CFLAGS) -c lidar_test.cpp $(LIB)


$(LIDAR_O): $(LIDAR_H)


clean:
	rm -rf *.o lidar