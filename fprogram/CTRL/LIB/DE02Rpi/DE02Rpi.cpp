/*!
 * \file DE02Rpi.cpp
 * \brief Class for communication between Rpi and DE0-Nano
 */

/* DOCUMENTATION
 * SPI lib wiringPi : http://wiringpi.com/reference/spi-library/
 * SPI and I2C in c++ details and examples : https://learn.sparkfun.com/tutorials/raspberry-pi-spi-and-i2c-tutorial/all
 */

/* ADDRESS ENC/ODO LEFT/RIGHT
 * 0x00 : Enc R
 * 0x01 : Enc L
 * 0x02 : Odo R
 * 0x03 : Odo L
 *
 * ADDRESS SONAR FRONT/BACK LEFT_RIGHT
 * 0x04 : FRONT R
 * 0x05 : FRONT L
 * 0x06 : BACK  R
 * 0x07 : BACK  L
 */

#include "DE02Rpi.h"


DE02Rpi::DE02Rpi()
{
    this->channel = 0; // CS0
    this->SPIspeed = 500000;
}

DE02Rpi::DE02Rpi(int channel,
                 int SPIspeed)
{
    this->channel = channel;
    this->SPIspeed = SPIspeed;
}

void DE02Rpi::init()
{
    wiringPiSPISetup(this->channel, this->SPIspeed);
}

unsigned char DE02Rpi::enc_address(int encoder, int left)
{
    unsigned char addr;

    if (encoder == 1) {
        if (left == 1) addr = 0x01;
        else addr = 0x00;
    } else {
        if (left == 1) addr = 0x03;
        else addr = 0x02;
    }
    return addr;
}

int DE02Rpi::enc_measure(int encoder, int left, int verbose = 0)
{
    unsigned char buffer[5];
    unsigned char addr = enc_address(encoder, left);
    buffer[0] = addr; buffer[1] = 0x00; buffer[2] = 0x00; buffer[3] = 0x00; buffer[4] = 0x00;

    wiringPiSPIDataRW(this->channel, buffer, 5);
    int count = buffer[4] + (buffer[3] << 8) + (buffer[2] << 16) + (buffer[1] << 24) - 4192;

    if (verbose) printf("DE02Rpi::enc_measure -> count = %d\n", count);
    return count;
}

unsigned char DE02Rpi::sonar_address(int front, int left)
{
    unsigned char addr;

    if (front == 1) {
        if (left == 1) addr = 0x05;
        else addr = 0x04;
    } else {
        if (left == 1) addr = 0x07;
        else addr = 0x06;
    }
    return addr;
}

int DE02Rpi::sonar_measure(int front, int left, int verbose = 0)
{
    unsigned char buffer[5];
    unsigned char addr = sonar_address(front, left);
    buffer[0] = addr; buffer[1] = 0x00; buffer[2] = 0x00; buffer[3] = 0x00; buffer[4] = 0x00;

    wiringPiSPIDataRW(this->channel, buffer, 5);
    int count = buffer[4] + (buffer[3] << 8) + (buffer[2] << 16) + (buffer[1] << 24) - 4192;

    if (verbose) printf("DE02Rpi::sonar_measure -> count = %d\n", count);
    return count;
}
