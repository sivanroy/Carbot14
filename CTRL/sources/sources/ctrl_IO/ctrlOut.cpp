/*!
 * \file ctrlOut.cpp
 * \brief Structures defining the outputs of the robot controller
 *
 * CAN Source for communication with socketCAN
 * https://github.com/craigpeacock/CAN-Examples/blob/master/cantransmit.c
 */

#include "ctrlOut.h"


void ctrlOut_init(ctrlOut *outputs)
{
    outputs->r_cmd = 0.0;
    outputs->l_cmd = 0.0;

    //outputs->can.init();
    can_init(outputs);
    motors_init(outputs);
}

void can_init(ctrlOut *outputs)
{
    system("sudo ifconfig can0 down");
    system("sudo ip link set can0 type can bitrate 125000");
    system("sudo ifconfig can0 up");
    usleep(1000);

    if ((outputs->s = socket(PF_CAN, SOCK_RAW, CAN_RAW)) < 0) {
        perror("Socket");
    }
    strcpy(outputs->ifr.ifr_name, "can0");

    if (ioctl(outputs->s, SIOCGIFINDEX, &(outputs->ifr)) < 0) {
        perror("ioctl");
    }
    memset(&(outputs->addr), 0, sizeof(outputs->addr));
    outputs->addr.can_family = AF_CAN;
    outputs->addr.can_ifindex = outputs->ifr.ifr_ifindex;
    if (bind(outputs->s, (struct sockaddr *)&(outputs->addr), sizeof(outputs->addr)) < 0) {
        perror("Bind");
    }
    outputs->frame.can_id = 0x708;
    outputs->frame.can_dlc = 3; // 3 bytes transmitted
}

void motors_init(ctrlOut *outputs)
{
    outputs->frame.data[0] = 0x1E;
    outputs->frame.data[1] = 0xFF;
    outputs->frame.data[2] = 0x40;

    if (write(outputs->s, &(outputs->frame), sizeof(struct can_frame)) != sizeof(struct can_frame)) {
        perror("Write LED ON");
    }
    printf("Write OK : LED ON\n");

    outputs->frame.data[0] = 0x1E;
    outputs->frame.data[1] = 0xFF;
    outputs->frame.data[2] = 0x00;

    if (write(outputs->s, &(outputs->frame), sizeof(struct can_frame)) != sizeof(struct can_frame)) {
        perror("Write LED OFF");
    }
    printf("Write OK : LED OFF\n");

    outputs->frame.data[0] = 0x1C;
    outputs->frame.data[1] = 0xFF;
    outputs->frame.data[2] = 0x80;

    if (write(outputs->s, &(outputs->frame), sizeof(struct can_frame)) != sizeof(struct can_frame)) {
        perror("Write INIT M_l");
    }
    printf("Write OK : INIT M_l\n");

    outputs->frame.data[0] = 0x1D;
    outputs->frame.data[1] = 0xFF;
    outputs->frame.data[2] = 0x80;

    if (write(outputs->s, &(outputs->frame), sizeof(struct can_frame)) != sizeof(struct can_frame)) {
        perror("Write INIT M_r");
    }
    printf("Write OK : INIT M_r\n");
}

void send_commands(ctrlStruct *cvs)
{
    ctrlOut *outputs;
    outputs = cvs->outputs;

    int cmd_l = outputs->l_cmd + 35;
    int cmd_r = outputs->r_cmd + 35;

    unsigned char cmd_l_hex = (unsigned char) cmd_l;
    unsigned char cmd_r_hex = (unsigned char) cmd_r;

    outputs->frame.data[0] = 0x25;
    outputs->frame.data[1] = 0xFF;
    outputs->frame.data[2] = cmd_l_hex;

    if (write(outputs->s, &(outputs->frame), sizeof(struct can_frame)) != sizeof(struct can_frame)) {
        perror("Write cmd_l");
    }

    outputs->frame.data[0] = 0x26;
    outputs->frame.data[1] = 0xFF;
    outputs->frame.data[2] = cmd_r_hex;

    if (write(outputs->s, &(outputs->frame), sizeof(struct can_frame)) != sizeof(struct can_frame)) {
        perror("Write cmd_r");
    }
}

void motors_stop(ctrlStruct *cvs)
{
    ctrlOut *outputs;
    outputs = cvs->outputs;

    outputs->frame.data[0] = 0x25;
    outputs->frame.data[1] = 0xFF;
    outputs->frame.data[2] = 0x23;

    if (write(outputs->s, &(outputs->frame), sizeof(struct can_frame)) != sizeof(struct can_frame)) {
        perror("Write STOP M_l");
    }

    outputs->frame.data[0] = 0x26;
    outputs->frame.data[1] = 0xFF;
    outputs->frame.data[2] = 0x23;

    if (write(outputs->s, &(outputs->frame), sizeof(struct can_frame)) != sizeof(struct can_frame)) {
        perror("Write STOP M_r");
    }
}

void can_free(ctrlStruct *cvs)
{
    ctrlOut *outputs;
    outputs = cvs->outputs;

    if (close(outputs->s) < 0) {
        perror("Close");
    }
}