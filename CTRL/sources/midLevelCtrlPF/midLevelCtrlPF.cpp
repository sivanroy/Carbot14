/*!
 * \file midLevelCtrlPF.cpp
 * \brief Middle-level controller of the robot
 */

#include "midLevelCtrlPF.h"
//#include <algorithm>    // std::max



void mlcPF_init(midLevelCtrlPF *mlcPF)
{
    mlcPF->dt = 0.01;

    mlcPF->sigma = .5;
    mlcPF->R_odo = 0.022;

    mlcPF->Kp_th = 8.0;
    mlcPF->max_th = 4;

    mlcPF->r_sp_ref = 0.0;
    mlcPF->l_sp_ref = 0.0;
    mlcPF->max_sp_ref = 15;
    mlcPF->min_sp_ref = -15;

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

    if(th_out>0) {
        th_out = fmin(th_out,mlcPF->max_th);
    } else {
        th_out = -1*fmin(-1*th_out,mlcPF->max_th);
    }

    double rout = v_out + th_out;
    double lout = v_out - th_out;

    if(rout>100 | lout>100) {
        rout/= fmax(abs(rout),abs(lout))*100;
        lout/= fmax(abs(rout),abs(lout))*100;
    }
    
    mlcPF->r_sp_ref = rout;
    mlcPF->l_sp_ref = lout;
}