/*!
 * \file myPosition.h
 * \brief Localization of the robot
 */

#ifndef CARBOT14_MYPOSITION_H
#define CARBOT14_MYPOSITION_H

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <math.h>

#include "../ctrlStruct/ctrlStruct.h"


typedef struct myPosition
{
    double dt; ///< time-step : time between 2 updates
    double R_odo; ///< radius of the odometers
    double b; ///< baseline of the robot

    double x; ///< position x
    double y; ///< position y
    double th; ///< orientation theta

    double v; ///< linear speed of the robot
    double w; ///< angular speed of the robot

} myPosition;

void mp_init(myPosition *mp);
void set_new_position(ctrlStruct *cvs);
void get_pos(ctrlStruct *cvs, double pos[5]);
double limit_angle(double th);
void update_pos(ctrlStruct *cvs);

#endif
