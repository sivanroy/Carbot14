/*!
 * \file lowLevelCtrl.h
 * \brief Low-level controller of the robot : PI with back-emf compensation, anti-windup and voltage saturation
 */

#ifndef CARBOT14_LOWLEVELCTRL_H
#define CARBOT14_LOWLEVELCTRL_H

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <math.h>

#include "../ctrlStruct/ctrlStruct.h"


typedef struct lowLevelCtrl
{
    double dt; ///< time-step : time between 2 updates
    double max; ///< upper bound of the output command
    double min; ///< lower bound of the output command
    double Kp; ///< proportional constant of the PI controller
    double Ki; ///< integral constant of the PI controller
    double r_integral_err; ///< integral error of the right wheel controller
    double l_integral_err; ///< integral error of the left wheel controller
    double kphiOnK;

} lowLevelCtrl;


void llc_init(lowLevelCtrl *llc);
void set_commands(ctrlStruct *cvs, double r_sp_ref, double l_sp_ref);


#endif
