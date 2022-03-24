//
// Created by Louis Libert on 18/03/22.
//
#include <errno.h>
#include <unistd.h>
#include <string.h>
#include <iostream>
#include <stdio.h>
#include <stdlib.h>

#include <net/if.h>
#include <sys/ioctl.h>
#include <sys/socket.h>

#include <linux/can.h>
#include <linux/can/raw.h>

int main() {

    int initCAN = 1;
    int startM = 1;
    int stopM = 1;

    int i = 48;
    unsigned char uc = (unsigned char)  i;
    printf("%x\n", uc);

    if (initCAN) {
        system("sudo ifconfig can0 down");
        system("sudo ip link set can0 type can bitrate 125000");
        system("sudo ifconfig can0 up");
        usleep(1000);
    }

    int s;

    if ((s = socket(PF_CAN, SOCK_RAW, CAN_RAW)) < 0) {
        perror("Socket");
        return 1;
    }


    struct ifreq ifr;

    strcpy(ifr.ifr_name, "can0");

    if (ioctl(s, SIOCGIFINDEX, &ifr) < 0) {
        perror("ioctl");
        return 1;
    }


    struct sockaddr_can addr;

    memset(&addr, 0, sizeof(addr));
    addr.can_family = AF_CAN;
    addr.can_ifindex = ifr.ifr_ifindex;
    if (bind(s, (struct sockaddr *)&addr, sizeof(addr)) < 0) {
        perror("Bind");
        return 1;
    }


    printf("Init OK\n");
    usleep(2000);


    struct can_frame frame;

    frame.can_id = 0x708;
    frame.can_dlc = 3; // 3 bytes transmitted
    frame.data[0] = 0x1E;
    frame.data[1] = 0xFF;
    frame.data[2] = 0x40;

    if (write(s, &frame, sizeof(struct can_frame)) != sizeof(struct can_frame)) {
        perror("Write LED ON");
        return 1;
    }
    printf("Write OK : LED ON\n");
    usleep(1000000);

    frame.data[0] = 0x1E;
    frame.data[1] = 0xFF;
    frame.data[2] = 0x00;

    if (write(s, &frame, sizeof(struct can_frame)) != sizeof(struct can_frame)) {
        perror("Write LED OFF");
        return 1;
    }
    printf("Write OK : LED OFF\n");


    if (startM) {

        frame.data[0] = 0x1C;
        frame.data[1] = 0xFF;
        frame.data[2] = 0x80;

        if (write(s, &frame, sizeof(struct can_frame)) != sizeof(struct can_frame)) {
            perror("Write INIT M_l");
            return 1;
        }
        printf("Write OK : INIT M_l\n");


        frame.data[0] = 0x1D;
        frame.data[1] = 0xFF;
        frame.data[2] = 0x80;

        if (write(s, &frame, sizeof(struct can_frame)) != sizeof(struct can_frame)) {
            perror("Write INIT M_r");
            return 1;
        }
        printf("Write OK : INIT M_r\n");


        frame.data[0] = 0x25;
        frame.data[1] = 0xFF;
        frame.data[2] = 0x0F;

        if (write(s, &frame, sizeof(struct can_frame)) != sizeof(struct can_frame)) {
            perror("Write start M_l");
            return 1;
        }
        printf("Write OK : start M_l\n");


        unsigned char speed = (unsigned char) 15;
        frame.data[0] = 0x26;
        frame.data[1] = 0xFF;
        frame.data[2] = speed;

        if (write(s, &frame, sizeof(struct can_frame)) != sizeof(struct can_frame)) {
            perror("Write start M_r");
            return 1;
        }
        printf("Write OK : start M_r\n");

        printf("sleep : 2s\n");
        usleep(2000000);
    }


    if (stopM) {

        frame.data[0] = 0x25;
        frame.data[1] = 0xFF;
        frame.data[2] = 0x23;

        if (write(s, &frame, sizeof(struct can_frame)) != sizeof(struct can_frame)) {
            perror("Write stop M_l");
            return 1;
        }
        printf("Write OK : stop M_l\n");


        frame.data[0] = 0x26;
        frame.data[1] = 0xFF;
        frame.data[2] = 0x23;

        if (write(s, &frame, sizeof(struct can_frame)) != sizeof(struct can_frame)) {
            perror("Write stop M_r");
            return 1;
        }
        printf("Write OK : stop M_r\n");
    }


    if (close(s) < 0) {
        perror("Close");
        return 1;
    }

    return 0;
}


