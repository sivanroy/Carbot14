/*!
 * \file ctrlIn.h
 * \brief Structure defining the inputs of the robot controller
 */

#ifndef _CTRLIN_H_
#define _CTRLIN_H_

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <math.h>
#include <wiringPiSPI.h>

#include "../ctrlStruct/ctrlStruct.h"
//#include "../DE02Rpi/DE02Rpi.h"


typedef struct ctrlStruct ctrlStruct;

typedef struct ctrlIn
{
    double t; ///< time reference [s]
    double dt; ///< time-step : time between 2 updates

    double radPerTick_enc; ///< radian per tick of the encoders
    double radPerTick_odo; ///< radian per tick of the odometers

    //DE02Rpi d2r; ///< class to communicate with DE0-Nano
    int d2r_channel;
    int d2r_speed;

    double r_sp_mes_enc; ///< right wheel speed [rad/s] : positive when the robot is going forward
    double l_sp_mes_enc; ///< left wheel speed [rad/s] : positive when the robot is going forward
    double r_sp_mes_odo; ///< right odometer speed [rad/s] : positive when the robot is going forward
    double l_sp_mes_odo; ///< left odometer speed [rad/s] : positive when the robot is going forward

} ctrlIn;

void ctrlIn_init(ctrlIn *inputs);
unsigned char d2r_enc_address(int encoder, int left);
int d2r_enc_measure(ctrlStruct *cvs, int encoder, int left, bool verbose);
void get_d2r_data(ctrlStruct *cvs);
void update_time(ctrlStruct *cvs);

#endif