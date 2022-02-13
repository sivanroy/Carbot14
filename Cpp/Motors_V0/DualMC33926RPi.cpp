//
// Created by Louis Libert on 8/11/21.
//

#include "DualMC33926RPi.h"

#define MAX_SPEED 480

DualMC33926RPi::DualMC33926RPi()
{
    //Pin map
    this->M1DIR = 24;//21
    this->M1PWM = 12;//26
    this->M2DIR = 25;//22
    this->M2PWM = 13;//23
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
    wiringPiSetupGpio();
    pwmSetMode(PWM_MODE_MS);
    pwmSetRange(MAX_SPEED);
    pwmSetClock(2);

    pinMode(this->M1PWM, PWM_OUTPUT);
    pinMode(this->M2PWM, PWM_OUTPUT);
    pinMode(this->M1DIR, OUTPUT);
    pinMode(this->M2DIR, OUTPUT);
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
        digitalWrite(this->M1DIR,HIGH);
    else
        digitalWrite(this->M1DIR,LOW);

    pwmWrite(this->M1PWM, speed);
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
        digitalWrite(this->M2DIR,HIGH);
    else
        digitalWrite(this->M2DIR,LOW);

    pwmWrite(this->M2PWM, speed);
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

