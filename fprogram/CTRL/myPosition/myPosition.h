/*!
 * \file myPosition.h
 * \brief Localization of the robot
 */

#ifndef _MYPOSITION_H_
#define _MYPOSITION_H_

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

} myPosition;

void mp_init(myPosition *mp);
void set_new_position(CtrlStruct *cvs);
double limit_angle(double th);

#endif
