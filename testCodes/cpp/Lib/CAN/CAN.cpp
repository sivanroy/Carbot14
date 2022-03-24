//
// Created by Louis Libert on 21/02/22.
//

#include "CAN.h"

CAN::CAN() {

    if ((this->s = socket(PF_CAN, SOCK_RAW, CAN_RAW)) < 0) {
        perror("Socket");
    }
    strcpy(this->ifr.ifr_name, "can0");

    if (ioctl(this->s, SIOCGIFINDEX, &(this->ifr)) < 0) {
        perror("ioctl");
    }
    memset(&(this->addr), 0, sizeof(this->addr));
    this->addr.can_family = AF_CAN;
    this->addr.can_ifindex = this->ifr.ifr_ifindex;
    if (bind(this->s, (struct sockaddr *)&(this->addr), sizeof(this->addr)) < 0) {
        perror("Bind");
    }
    this->frame.can_id = 0x708;
    this->frame.can_dlc = 3; // 3 bytes transmitted
}

void CAN::init()
{
    /*!
     * CAN UP
     */
    system("sudo ifconfig can0 down");
    system("sudo ip link set can0 type can bitrate 125000");
    system("sudo ifconfig can0 up");
    usleep(1000);

    /*!
     * LED ON -> OFF
     */
    this->frame.data[0] = 0x1E;
    this->frame.data[1] = 0xFF;
    this->frame.data[2] = 0x40;

    if (write(this->s, &(this->frame), sizeof(struct can_frame)) != sizeof(struct can_frame)) {
        perror("Write LED ON");
    }
    printf("Write OK : LED ON\n");

    this->frame.data[0] = 0x1E;
    this->frame.data[1] = 0xFF;
    this->frame.data[2] = 0x00;

    if (write(this->s, &(this->frame), sizeof(struct can_frame)) != sizeof(struct can_frame)) {
        perror("Write LED OFF");
    }
    printf("Write OK : LED OFF\n");

    /*!
     * Motors INIT
     */
    this->frame.data[0] = 0x1C;
    this->frame.data[1] = 0xFF;
    this->frame.data[2] = 0x80;

    if (write(this->s, &(this->frame), sizeof(struct can_frame)) != sizeof(struct can_frame)) {
        perror("Write INIT M_l");
    }
    printf("Write OK : INIT M_l\n");

    this->frame.data[0] = 0x1D;
    this->frame.data[1] = 0xFF;
    this->frame.data[2] = 0x80;

    if (write(this->s, &(this->frame), sizeof(struct can_frame)) != sizeof(struct can_frame)) {
        perror("Write INIT M_r");
    }
    printf("Write OK : INIT M_r\n");
}

void CAN::setSpeeds(int cmd_l, int cmd_r)
{
    cmd_l = cmd_l + 35;
    cmd_r = cmd_r + 35;

    unsigned char cmd_l_hex = (unsigned char) cmd_l;
    unsigned char cmd_r_hex = (unsigned char) cmd_r;

    this->frame.data[0] = 0x25;
    this->frame.data[1] = 0xFF;
    this->frame.data[2] = cmd_l_hex;

    if (write(this->s, &(this->frame), sizeof(struct can_frame)) != sizeof(struct can_frame)) {
        perror("Write STOP M_l");
    }

    this->frame.data[0] = 0x26;
    this->frame.data[1] = 0xFF;
    this->frame.data[2] = cmd_r_hex;

    if (write(this->s, &(this->frame), sizeof(struct can_frame)) != sizeof(struct can_frame)) {
        perror("Write STOP M_r");
    }

}

void CAN::stop()
{
    this->frame.data[0] = 0x25;
    this->frame.data[1] = 0xFF;
    this->frame.data[2] = 0x23;

    if (write(this->s, &(this->frame), sizeof(struct can_frame)) != sizeof(struct can_frame)) {
        perror("Write STOP M_l");
    }

    this->frame.data[0] = 0x26;
    this->frame.data[1] = 0xFF;
    this->frame.data[2] = 0x23;

    if (write(this->s, &(this->frame), sizeof(struct can_frame)) != sizeof(struct can_frame)) {
        perror("Write STOP M_r");
    }
}

void CAN::freeCAN()
{
    if (close(this->s) < 0) {
        perror("Close");
    }
}