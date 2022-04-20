/*!
 * \file ctrlIn.h
 * \brief Structure defining the inputs of the robot controller
 */

#ifndef CARBOT14_CTRLIN_H
#define CARBOT14_CTRLIN_H

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <math.h>
#include <wiringPiSPI.h>

#include "../ctrlStruct/ctrlStruct.h"


typedef struct ctrlStruct ctrlStruct;

typedef struct ctrlIn
{
    double t; ///< time reference [s]
    double dt; ///< time-step : time between 2 updates

    double radPerTick_enc; ///< radian per tick of the encoders
    double radPerTick_odo; ///< radian per tick of the odometers

    int d2r_channel;
    int d2r_speed;

    double r_sp_mes_enc; ///< right wheel speed [rad/s] : positive when the robot is going forward
    double l_sp_mes_enc; ///< left wheel speed [rad/s] : positive when the robot is going forward
    double r_sp_mes_odo; ///< right odometer speed [rad/s] : positive when the robot is going forward
    double l_sp_mes_odo; ///< left odometer speed [rad/s] : positive when the robot is going forward

    double r_front_s;
    double l_front_s;
    double r_back_s;
    double l_back_s;

    int team;
    int start;

    int leftWheelBlocked;
    int rightWheelBlocked;

} ctrlIn;

void ctrlIn_init(ctrlIn *inputs);

typedef struct lowPassFilter{
    double value_le;
    double value_re;
    double value_lo;
    double value_ro;
    double beta;
}lowPassFilter;

void lpf_init(lowPassFilter *lpf);
double lpf(ctrlStruct *cvs,double val, int select);


unsigned char d2r_enc_address(int encoder, int left, int sonarF);
int d2r_enc_measure(ctrlStruct *cvs, int encoder, int left, int sonarF ,bool verbose);
void get_d2r_data(ctrlStruct *cvs);

void update_time(ctrlStruct *cvs);

#endif
