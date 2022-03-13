/*!
 * \file ctrlOut.cpp
 * \brief Structures defining the outputs of the robot controller
 */

#include "ctrlOut.h"


void ctrlOut_init(ctrlOut *outputs)
{
    outputs->r_cmd = 0.0;
    outputs->l_cmd = 0.0;

    outputs->can.init();
}

void send_commands(ctrlStruct *cvs)
{
    ctrlOut *outputs;
    outputs = cvs->outputs;

    outputs->can.motor_commands(outputs->r_cmd, outputs->l_cmd);
}