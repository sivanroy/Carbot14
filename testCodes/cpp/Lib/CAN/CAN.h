//
// Created by Louis Libert on 21/02/22.
//

#ifndef CARBOT14_CAN_H
#define CARBOT14_CAN_H

#include <errno.h>
#include <unistd.h>
#include <string.h>
#include <iostream>
#include <sstream>
#include <stdio.h>
#include <stdlib.h>

#include <net/if.h>
#include <sys/ioctl.h>
#include <sys/socket.h>

#include <linux/can.h>
#include <linux/can/raw.h>

class CAN
{
public:
    CAN();
    void init();
    void setSpeeds(int cmd_l, int cmd_r);
    void stop();
    void freeCAN();
private:
    int s;
    struct sockaddr_can addr;
    struct ifreq ifr;
    struct can_frame frame;
};


#endif //CARBOT14_CAN_H
