//
// Created by Louis Libert on 8/11/21.
//

#ifndef CARBOT14_DUALMC33926RPI_H
#define CARBOT14_DUALMC33926RPI_H

#include <wiringPi.h>
#include <softPwm.h>
#include <unistd.h>

class DualMC33926RPi
{
public:
        // CONSTRUCTORS
        // Default pin selection.
        DualMC33926RPi();

        // User-defined pin selection.
        DualMC33926RPi(unsigned char M1DIR,
                       unsigned char M1PWM,
                       unsigned char M2DIR,
                       unsigned char M2PWM);

        // PUBLIC METHODS
        void init(); // Initialize TIMER 1, set the PWM to 20kHZ.
        void setM1Speed(int speed); // Set speed for M1.
        void setM2Speed(int speed); // Set speed for M2.
        void setSpeeds(int m1Speed, int m2Speed); // Set speed for both M1 and M2.
        void stop();

private:
    unsigned char M1DIR;
    unsigned char M2DIR;
    unsigned char M1PWM;
    unsigned char M2PWM;

};

#endif //CARBOT14_DUALMC33926RPI_H