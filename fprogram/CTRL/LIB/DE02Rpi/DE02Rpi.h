/*!
 * \file DE02Rpi.h
 * \brief Class for communication between Rpi and DE0-Nano
 */

#ifndef _DE02RPI_H_
#define _DE02RPI_H_

#include <iostream>
#include <errno.h>
#include <unistd.h>
#include <wiringPiSPI.h>


class DE02Rpi
{
public:
    // CONSTRUCTORS
    // Default pin selection.
    DE02Rpi();

    // User-defined pin selection.
    DE02Rpi(int channel,
            int SPIspeed);

    // PUBLIC METHODS
    void init(); // Initialize SPI bus (channel [0 or 1], SPIspeed [Hz])
    unsigned char enc_address(int encoder, int left);
    int enc_measure(int encoder, int left, int verbose);
    unsigned char sonar_address(int front, int left);
    int sonar_measure(int front, int left, int verbose);

private:
    int channel;
    int SPIspeed;

};

#endif
