/*!
 * \file ctrlIn.cpp
 * \brief Structures defining the inputs of the robot controller
 */

#include "ctrlIn.h"


void ctrlIn_init(ctrlIn *inputs)
{
    inputs->t = 0.0;
    inputs->dt = 0.01;

    inputs->d2r.init();

    inputs->r_sp_mes = 0.0;
    inputs->l_sp_mes = 0.0;
}

void get_speeds_mes(ctrlStruct *cvs)
{
    ctrlIn *inputs;

    inputs = cvs->inputs;
}