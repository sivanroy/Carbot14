/*!
 * \file ctrlIn.cpp
 * \brief Structures defining the inputs of the robot controller
 */

#include "ctrlIn.h"


void ctrlIn_init(ctrlIn *inputs)
{
    inputs->t = 0.0;
    inputs->dt = 0.01;

    inputs->radPerTick_enc = 2 * M_PI/(8192 * 19);
    inputs->radPerTick_odo = 2 * M_PI/8192;

    inputs->d2r.init();

    inputs->r_sp_mes_enc = 0.0;
    inputs->l_sp_mes_enc = 0.0;
    inputs->r_sp_mes_odo = 0.0;
    inputs->l_sp_mes_odo = 0.0;
}

void get_speeds_mes(ctrlStruct *cvs)
{
    ctrlIn *inputs;
    inputs = cvs->inputs;

    double dt = inputs->dt;
    double rpt_enc = inputs->radPerTick_enc;
    double rpt_odo = inputs->radPerTick_odo;

    int r_ticks_enc = inputs->d2r.enc_measure(1, 0);
    int l_ticks_enc = inputs->d2r.enc_measure(1, 1);
    int r_ticks_odo = inputs->d2r.enc_measure(0, 0);
    int l_ticks_odo = inputs->d2r.enc_measure(0, 1);

    inputs->r_sp_mes_enc = - r_ticks_enc * rpt_enc/dt;
    inputs->l_sp_mes_enc =   l_ticks_enc * rpt_enc/dt;
    inputs->r_sp_mes_odo = - r_ticks_odo * rpt_odo/dt;
    inputs->l_sp_mes_odo =   l_ticks_odo * rpt_odo/dt;
}

void update_time(ctrlStruct *cvs)
{
    ctrlIn *inputs;
    inputs = cvs->inputs;

    inputs->t += inputs->dt;
}