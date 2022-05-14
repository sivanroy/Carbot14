/*
----------------------------
Welcome to the midleLevelCtrlPF.h
----------------------------
Midle level controller of the hlcPF (potential field)
Get a reference speed and an absolute angle , return the wheels
reference speed
-----------------------------
*/

#ifndef CARBOT14_MIDLEVELCTRLPF_H
#define CARBOT14_MIDLEVELCTRLPF_H

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <math.h>

#include "../ctrlStruct/ctrlStruct.h"


typedef struct midLevelCtrlPF
{
    double dt; ///< time-step : time between 2 updates

    double sigma;
    double R_odo; ///< radius of the odometers

    double Kp_th; ///< proportional constant of the PI 'th' controller
    double max_th;
    double K_orient;

    double r_sp_ref; ///< reference speed of the right wheel
    double l_sp_ref; ///< reference speed of the left wheel
    double max_sp_ref; ///< upper bound of the reference speed
    double min_sp_ref; ///< lower bound of the reference speed

    double t_start;

} midLevelCtrlPF;

void mlcPF_init(midLevelCtrlPF *mlcPF);
void mlcPF_out(ctrlStruct *cvs, double v_ref, double th_ref);

#endif
