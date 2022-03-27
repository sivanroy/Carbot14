/*!
 * \file midLevelCtrl_gr5.cc
 * \brief Middle level controller : find speed ref to reach goal position
 */
#include <stdio.h>
#include <math.h>

#include "midLevelCtrl.h"


void init_midLevelCtrl(midLevelCtrl *mlc)
{
    mlc->dt = 0.001;

    mlc->d_ref = 0.0;
    mlc->d_mes = 0.0;
    mlc->th_ref = 0.0;
    mlc->th_mes = 0.0;

    mlc->Kp_d = 15.0;
    mlc->Ki_d = 0.0;
    mlc->integral_err_d = 0.0;

    mlc->Kp_th = 25.0;
    mlc->Ki_th = 0.0;
    mlc->integral_err_th = 0.0;

    mlc->r_sp_ref = 0.0;
    mlc->l_sp_ref = 0.0;
    mlc->max_sp_ref = 5.0;
    mlc->min_sp_ref = -5.0;

    mlc->reach_goal = 0;
}

void set_d_th_ref_mes(ctrlStruct *cvs, double x_g, double y_g)
{
    // structures declaration
    myPosition *mp;
    midLevelCtrl *mlc;

    // structures initialization
    mp  = cvs->mp;
    mlc = cvs->mlc;

    double x_diff = x_g - mp->x;
    double y_diff = y_g - mp->y;

    mlc->d_ref = 0.0;
    mlc->th_ref = limit_angle(atan2(y_diff, x_diff));

    // mes
    mlc->d_mes = sqrt(pow(x_diff, 2) + pow(y_diff, 2));
    mlc->th_mes = mp->th;

    // goal reached ?
    mlc->reach_goal = 0;
    double near_g = 0.02;
    if ( sqrt(x_diff*x_diff + y_diff*y_diff) < near_g) mlc->reach_goal = 1;
}

void set_speed_ref(ctrlStruct *cvs, double x_g, double y_g)
{
    set_d_th_ref_mes(cvs, x_g, y_g);

    // structure declaration
    midLevelCtrl *mlc;

    // structure initialization
    mlc = cvs->mlc;

    // calculate errors
    double d_error = mlc->d_mes - mlc->d_ref;
    double th_error = mlc->th_mes - mlc->th_ref;
    if(th_error> 3.1415) {
        th_error-=3.1415*2;
    } else if(th_error < -3.1415) {
        th_error += 3.1415*2;
    }

    // proportional terms
    double d_Pout = mlc->Kp_d * d_error;
    double th_Pout = mlc->Kp_th * th_error;

    // integral terms
    mlc->integral_err_d += d_error * mlc->dt;
    mlc->integral_err_th += th_error * mlc->dt;
    double d_Iout = mlc->Ki_d * mlc->integral_err_d;
    double th_Iout = mlc->Ki_th * mlc->integral_err_th;

    // calculate total outputs (commands)
    double d_out = d_Pout + d_Iout;
    double th_out = th_Pout + th_Iout;

    // find reference speeds
    double r_sp_ref = d_out - th_out;
    double l_sp_ref = d_out + th_out;

    // Restrict to max/min
    if (r_sp_ref > mlc->max_sp_ref) r_sp_ref = mlc->max_sp_ref;
    else if (r_sp_ref < mlc->min_sp_ref) r_sp_ref = mlc->min_sp_ref;

    if (l_sp_ref > mlc->max_sp_ref) l_sp_ref = mlc->max_sp_ref;
    else if (l_sp_ref < mlc->min_sp_ref) l_sp_ref = mlc->min_sp_ref;

    // set reference speeds
    if (mlc->reach_goal) {
        mlc->r_sp_ref = 0.0;
        mlc->l_sp_ref = 0.0;
    } else {
        mlc->r_sp_ref = r_sp_ref;
        mlc->l_sp_ref = l_sp_ref;
    }

}

