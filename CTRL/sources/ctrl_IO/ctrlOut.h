/*
----------------------------
Welcome to the ctrlOut.h
----------------------------
Structure defining outputs of the robot controller
-----------------------------
*/

#ifndef CARBOT14_CTRLOUT_H
#define CARBOT14_CTRLOUT_H

#include <errno.h>
#include <unistd.h>
#include <string.h>
#include <iostream>
#include <sstream>
#include <stdio.h>
#include <stdlib.h>
#include <math.h>

#include <net/if.h>
#include <sys/ioctl.h>
#include <sys/socket.h>

#include <linux/can.h>
#include <linux/can/raw.h>

#include "../ctrlStruct/ctrlStruct.h"


typedef struct ctrlStruct ctrlStruct;

typedef struct ctrlOut
{
    int r_cmd; ///< Command [-] of the right wheel : bounded in [-35 ; 35]
    int l_cmd; ///< Command [-] of the left wheel : bounded in [-35 ; 35]

    //CAN can; ///< class to control motors with CAN bus
    int s;
    struct sockaddr_can addr;
    struct ifreq ifr;
    struct can_frame frame;

} ctrlOut;

void ctrlOut_init(ctrlOut *outputs);
void can_init(ctrlOut *outputs);
void motors_init(ctrlOut *outputs);
void send_commands(ctrlStruct *cvs);
void motors_stop(ctrlStruct *cvs);
void can_free(ctrlStruct *cvs);

#endif
