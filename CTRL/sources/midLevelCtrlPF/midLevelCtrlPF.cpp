/*!
 * \file midLevelCtrlPF.cpp
 * \brief Middle-level controller of the robot
 */

#include "midLevelCtrlPF.h"


void mlcPF_init(midLevelCtrlPF *mlcPF)
{
    mlcPF->dt = 0.01;

    mlcPF->sigma = .45;
    mlcPF->R_odo = 0.022;

    mlcPF->Kp_th = 3.0;

    mlcPF->r_sp_ref = 0.0;
    mlcPF->l_sp_ref = 0.0;
    mlcPF->max_sp_ref = 5.0;
    mlcPF->min_sp_ref = -5.0;

    mlcPF->t_start = 0;
}

void mlcPF_out(ctrlStruct *cvs, double v_ref, double th_ref)
{
    // structure declaration
    midLevelCtrlPF *mlcPF;
    myPosition *mp;

    // structure initialization
    mlcPF = cvs->mlcPF;
    mp = cvs->mp;

    // calculate th error
    double th_error = limit_angle(th_ref - mp->th);

    // proportional terms
    double v_out = (v_ref/mlcPF->R_odo) * exp(-pow(th_error/mlcPF->sigma, 2));
    double th_out = mlcPF->Kp_th * th_error;

    // Restrict to max/min
    if (th_out > mlcPF->max_sp_ref) th_out = mlcPF->max_sp_ref;
    else if (th_out < mlcPF->min_sp_ref) th_out = mlcPF->min_sp_ref;

    if (v_out > mlcPF->max_sp_ref - abs(th_out)) v_out = mlcPF->max_sp_ref - abs(th_out);
    else if (v_out < mlcPF->min_sp_ref + abs(th_out)) v_out = mlcPF->min_sp_ref + abs(th_out);

    mlcPF->r_sp_ref = v_out + th_out;
    mlcPF->l_sp_ref = v_out - th_out;
}