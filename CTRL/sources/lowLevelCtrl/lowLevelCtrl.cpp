/*!
 * \file lowLevelCtrl.cpp
 * \brief Low-level controller of the robot : PI with back-emf compensation, anti-windup and voltage saturation
 */

#include "lowLevelCtrl.h"


void llc_init(lowLevelCtrl *llc)
{
    llc->dt = 0.01;
    llc->max = 35;
    llc->min = -35;

    llc->Kp = 0.706;
    llc->Ki = 0;//39.437;

    llc->r_integral_err = 0.0;
    llc->l_integral_err = 0.0;
    llc->kphiOnK = 0.23*(35/24);
}

void set_commands(ctrlStruct *cvs, double r_sp_ref, double l_sp_ref)
{
    // variables declaration
    ctrlIn  *inputs;
    ctrlOut *outputs;
    lowLevelCtrl *llc;

    // variables initialization
    inputs  = cvs->inputs;
    outputs = cvs->outputs;
    llc  = cvs->llc;

    double r_sp_mes = inputs->r_sp_mes_enc;
    double l_sp_mes = inputs->l_sp_mes_enc;

    //kphi term
    double kphi_r = r_sp_mes * llc->kphiOnK;
    double kphi_l = l_sp_mes * llc->kphiOnK;

    // calculate errors
    double r_error = r_sp_ref - r_sp_mes;
    double l_error = l_sp_ref - l_sp_mes;

    // proportional terms
    double r_Pout = llc->Kp * r_error;
    double l_Pout = llc->Kp * l_error;

    // integral terms
    double r_int_err = llc->r_integral_err + r_error * llc->dt;
    double l_int_err = llc->l_integral_err + l_error * llc->dt;

    double r_Iout = llc->Ki * r_int_err;
    double l_Iout = llc->Ki * l_int_err;

    // calculate total outputs (commands)
    double r_cmd = r_Pout + r_Iout + kphi_r;
    double l_cmd = l_Pout + l_Iout + kphi_l;

    // Restrict to max/min
    if (r_cmd > llc->max) r_cmd = llc->max;
    else if (r_cmd < llc->min) r_cmd = llc->min;
    else llc->r_integral_err = r_int_err;

    if (l_cmd > llc->max) l_cmd = llc->max;
    else if (l_cmd < llc->min) l_cmd = llc->min;
    else llc->l_integral_err = l_int_err;

    // wheel commands
    if (r_sp_ref == 0.0) {
        r_cmd = 0.0;
        llc->r_integral_err = 0.0;
    }
    if (l_sp_ref == 0.0) {
        l_cmd = 0.0;
        llc->l_integral_err = 0.0;
    }
    outputs->r_cmd = r_cmd;
    outputs->l_cmd = l_cmd;
}