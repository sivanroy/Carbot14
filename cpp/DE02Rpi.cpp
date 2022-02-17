//
// Created by Louis Libert on 15/02/22.
//

/* DOCUMENTATION
 * SPI lib wiringPi : http://wiringpi.com/reference/spi-library/
 * SPI and I2C in c++ details and examples : https://learn.sparkfun.com/tutorials/raspberry-pi-spi-and-i2c-tutorial/all
 */

/* ADDRESS ENC/ODO LEFT/RIGHT
 * 0x00 : Enc R
 * 0x01 : Enc L
 * 0x02 : Odo R
 * 0x03 : Odo L
 */

/* USE THE CLASS
 * DE02Rpi DE02Rpi;                           // create class
 * DE02Rpi.init();                            // init SPI bus
 * int count = DE02Rpi.measure(encoder,left); // return count of enc/odo (encoder = 1/0) left/right (left = 1/0)
 */

#include "DE02Rpi.h"

using namespace std;


DE02Rpi::DE02Rpi()
{
    this->channel = 1;
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

unsigned char DE02Rpi::giveAddress(int encoder, int left)
{
    unsigned char addr;

    if (encoder == 1) {
        if (left == 1) addr = 0x00;
        else addr = 0x01;
    } else {
        if (left == 1) addr = 0x03;
        else addr = 0x02;
    }
    return addr;
}

int DE02Rpi::measure(int encoder, int left)
{
    unsigned char buffer[5];
    unsigned char addr = giveAddress(encoder, left);
    buffer[0] = addr; buffer[1] = 0x00; buffer[2] = 0x00; buffer[3] = 0x00; buffer[4] = 0x00;

    wiringPiSPIDataRW(this->channel, buffer, 5);
    int count = buffer[4] + (buffer[3] << 8) + (buffer[2] << 16) + (buffer[1] << 24) - 4192;

    //printf("DE02Rpi : data measure\n");
    return count;
}
