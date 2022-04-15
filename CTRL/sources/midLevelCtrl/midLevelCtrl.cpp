/*!
 * \file midLevelCtrl_gr5.cc
 * \brief Middle level controller : find speed ref to reach goal position
 */
#include <stdio.h>
#include <math.h>

#include "midLevelCtrl.h"


void init_midLevelCtrl(midLevelCtrl *mlc)
{
    mlc->dt = 0.01;

    mlc->d_ref = 0.0;
    mlc->d_mes = 0.0;
    mlc->th_ref = 0.0;
    mlc->th_mes = 0.0;

    mlc->Kp_d = 70.0;
    mlc->sigma = 0.12;
    mlc->Ki_d = 0.0;
    mlc->integral_err_d = 0.0;
    mlc->max_th = 4;

    mlc->Kp_th = 4.0;
    mlc->Ki_th = 0.0;
    mlc->integral_err_th = 0.0;

    mlc->r_sp_ref = 0.0;
    mlc->l_sp_ref = 0.0;
    mlc->max_sp_ref = 8;
    mlc->min_sp_ref = -8;

    mlc->reach_goal = 0;
    mlc->d = 0;

    //double values[200];
    mlc->size = 200; // !!  à la size < 200
    mlc->pointer = 0;
    mlc->val_err = 0.01;
    for (int i = 0; i< mlc->size; i++) {
        mlc->values[i] = -1;
    }

    mlc->error = 0.05;
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
    mlc->d = sqrt(x_diff*x_diff + y_diff*y_diff);
    if (mlc->d < mlc->error) mlc->reach_goal = 1;
}

void set_speed_ref(ctrlStruct *cvs, double x_g, double y_g, int goForward)
{
    set_d_th_ref_mes(cvs, x_g, y_g);

    midLevelCtrl *mlc = cvs->mlc;

    // calculate errors
    double d_error = mlc->d_mes - mlc->d_ref;
    double th_error = limit_angle(mlc->th_mes - mlc->th_ref);

    //toCheck
    if (!goForward){
        d_error = -d_error;
        th_error = limit_angle(th_error+M_PI);
    }

    // proportional terms
    double d_Pout = mlc->Kp_d * d_error * exp(-pow(th_error/mlc->sigma, 2));
    double th_Pout = mlc->Kp_th * th_error;

    // integral terms
    mlc->integral_err_d += d_error * mlc->dt;
    mlc->integral_err_th += th_error * mlc->dt;
    double d_Iout = mlc->Ki_d * mlc->integral_err_d;
    double th_Iout = mlc->Ki_th * mlc->integral_err_th;

    // calculate total outputs (commands)
    double d_out = d_Pout + d_Iout;
    double th_out = th_Pout + th_Iout;

    if(th_out>0) {
        th_out = fmin(th_out,mlc->max_th);
    } else {
        th_out = -1*fmin(-1*th_out,mlc->max_th);
    }

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
        if(abs(r_sp_ref)>mlc->max_sp_ref | abs(l_sp_ref)>mlc->max_sp_ref) {
            //int r1 = 1; int l1 = 1;
            //if(r_sp_ref<0) r1 = -1;
            //if(l_sp_ref<0) l1 = -1;
            r_sp_ref = r_sp_ref / fmax(abs(r_sp_ref),abs(l_sp_ref))*mlc->max_sp_ref;
            l_sp_ref = l_sp_ref / fmax(abs(r_sp_ref),abs(l_sp_ref))*mlc->max_sp_ref;
        }  
        mlc->r_sp_ref = r_sp_ref;
        mlc->l_sp_ref = l_sp_ref;
    }
    fprintf(cvs->mlc_data, "%f,%f,%f,%f,%f,%f,%f\n", cvs->inputs->t, mlc->r_sp_ref, mlc->l_sp_ref, mlc->d_ref, mlc->d_mes, mlc->th_ref, mlc->th_mes);

}

int check_blocked(ctrlStruct *cvs){
    int pointer = cvs->mlc->pointer;
    int size  = cvs->mlc->size;
    double *values = cvs->mlc->values;
    pointer += 1;
    pointer %= size;
    values[pointer] = cvs->mlc->d;

    double dif = 0;
    for (int i = 0; i < size; i ++) {
        int p = (pointer+i)%size;
        int pp = (pointer+i+1)%size;
        dif += abs(values[p]-values[pp]);
    }
    if (dif < cvs->mlc->val_err){
        return 1;
    }
    return 0;
}

