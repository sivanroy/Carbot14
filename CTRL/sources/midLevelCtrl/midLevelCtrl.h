/*
----------------------------
Welcome to the midleLevelCtrl.h
----------------------------
structures and functions for a midle level controller
which input are coordinate and output wheels speed references
/!\ this is not the hlcPF mlc. The folder of it is midLevelCtrlPF
-----------------------------
*/

#ifndef _MIDLEVELCTRL_GR5_H_ 
#define _MIDLEVELCTRL_GR5_H_ 

#include "../ctrlStruct/ctrlStruct.h"


typedef struct midLevelCtrl
{
    double dt; ///< time-step : time between 2 updates

    double d_ref; ///< distance of reference
    double d_mes; ///< measured distance
    double th_ref; ///< theta of reference
    double th_mes; ///< measured theta
    double sigma;
    double max_th;

    double Kp_d; ///< proportional constant of the PI 'd' controller
    double Ki_d; ///< integral constant of the PI 'd' controller
    double integral_err_d; ///< integral error of the 'd' controller

    double Kp_th; ///< proportional constant of the PI 'th' controller
    double Ki_th; ///< integral constant of the PI 'th' controller
    double integral_err_th; ///< integral error of the 'th' controller

    double r_sp_ref; ///< reference speed of the right wheel
    double l_sp_ref; ///< reference speed of the left wheel
    double max_sp_ref; ///< upper bound of the reference speed
    double min_sp_ref; ///< lower bound of the reference speed

    int reach_goal; ///< 1 if goal reached, 0 if not
    double d;

    double error;

} midLevelCtrl;

void init_midLevelCtrl(midLevelCtrl *mlc);
void set_d_th_ref_mes(ctrlStruct *cvs, double x_g, double y_g);
void set_speed_ref(ctrlStruct *cvs, double x_g, double y_g, int goForward = 1);

#endif 