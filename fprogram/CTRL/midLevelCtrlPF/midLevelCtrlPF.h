/*!
 * \file midLevelCtrlPF.h
 * \brief Middle-level controller of the robot for Potential Field method
 */

#ifndef _MIDLEVELCTRLPF_H_
#define _MIDLEVELCTRLPF_H_

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <math.h>


typedef struct midLevelCtrlPF
{
    double dt; ///< time-step : time between 2 updates

    double sigma;
    double R_odo; ///< radius of the odometers

    double Kp_th; ///< proportional constant of the PI 'th' controller

    double r_sp_ref; ///< reference speed of the right wheel
    double l_sp_ref; ///< reference speed of the left wheel
    double max_sp_ref; ///< upper bound of the reference speed
    double min_sp_ref; ///< lower bound of the reference speed

} midLevelCtrlPF;

void mlcPF_init(midLevelCtrlPF *mlcPF);
void mlcPF_out(CtrlStruct *cvs, double v_ref, double th_ref);

#endif
