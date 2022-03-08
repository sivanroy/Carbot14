/*!
 * \file ctrlOut.cpp
 * \brief Structures defining the outputs of the robot controller
 */

#include "ctrlOut.h"


void ctrlOut_init(ctrlOut *outputs)
{
    outputs->r_cmd = 0.0;
    outputs->l_cmd = 0.0;
}