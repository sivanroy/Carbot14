//
// Created by Louis Libert on 8/11/21.
//

/*
 * Links to wiringPi/PWM understanding :
 * - http://wiringpi.com/examples/gertboard-and-wiringpi/blink/ : example of wiringPi usage for a LED
 * - http://wiringpi.com/reference/software-pwm-library/ : PWM usage
 */

#include "DualMC33926RPi.h"

#define MAX_SPEED 100
#define PWM_RANGE 100

DualMC33926RPi::DualMC33926RPi()
{
    //Pin map
    this->M1DIR = 21;//24
    this->M1PWM = 26;//12
    this->M2DIR = 22;//25
    this->M2PWM = 23;//13
}

DualMC33926RPi::DualMC33926RPi(unsigned char M1DIR,
                               unsigned char M1PWM,
                               unsigned char M2DIR,
                               unsigned char M2PWM)
{
    this->M1DIR = M1DIR;
    this->M1PWM = M1PWM;
    this->M2DIR = M2DIR;
    this->M2PWM = M2PWM;
}

void DualMC33926RPi::init()
{
    wiringPiSetup(); // or wiringPiSetupGpio() (pin : 5 12 6 13), or wiringPiSetupPhys()
    
    // Direction : 1 or 0
    pinMode(this->M1DIR, OUTPUT);
    pinMode(this->M2DIR, OUTPUT);

    // PWM
    softPwmCreate(this->M1PWM, 0, PWM_RANGE);
    softPwmCreate(this->M2PWM, 0, PWM_RANGE);
}

void DualMC33926RPi::setM1Speed(int speed)
{
    unsigned char reverse = 0;

    if (speed < 0)
    {
        speed = -speed;  // Make speed a positive quantity
        reverse = 1;  // Preserve the direction
    }
    if (speed > MAX_SPEED)  // Max PWM dutycycle
        speed = MAX_SPEED;


    if (reverse)
        digitalWrite(this->M1DIR,LOW);
    else
        digitalWrite(this->M1DIR,HIGH);

    softPwmWrite(this->M1PWM, speed);
}

void DualMC33926RPi::setM2Speed(int speed)
{
    unsigned char reverse = 0;

    if (speed < 0)
    {
        speed = -speed;  // Make speed a positive quantity
        reverse = 1;  // Preserve the direction
    }
    if (speed > MAX_SPEED)  // Max PWM dutycycle
        speed = MAX_SPEED;


    if (reverse)
        digitalWrite(this->M2DIR,LOW);
    else
        digitalWrite(this->M2DIR,HIGH);

    softPwmWrite(this->M2PWM, speed);
}

// Set speed for motor 1 and 2
void DualMC33926RPi::setSpeeds(int m1Speed, int m2Speed)
{
    setM1Speed(m1Speed);
    setM2Speed(m2Speed);
}

void DualMC33926RPi::stop()
{
    setSpeeds(0, 0);
}

