//
// Created by Louis Libert on 15/02/22.
//

#ifndef CARBOT14_DE02RPI_H
#define CARBOT14_DE02RPI_H

#include <iostream>
#include <errno.h>
#include <wiringPiSPI.h>
#include <unistd.h>

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
    unsigned char giveAddress(int encoder, int left);
    int measure(int encoder, int left);

private:
    int channel;
    int SPIspeed;

};

#endif //CARBOT14_DE02RPI_H
