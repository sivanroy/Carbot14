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


typedef struct ctrlIn
{
    double t; ///< time reference [s]

    double r_sp_mes; ///< right wheel speed [rad/s] : positive when the robot is going forward
    double l_sp_mes; ///< left wheel speed [rad/s] : positive when the robot is going forward

} ctrlIn;

void ctrlIn_init(ctrlIn *inputs);

#endif
