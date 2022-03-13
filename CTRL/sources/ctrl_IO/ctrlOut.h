/*!
 * \file ctrlOut.h
 * \brief Structure defining the outputs of the robot controller
 */

#ifndef _CTRLOUT_H_
#define _CTRLOUT_H_

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <math.h>

#include "../ctrlStruct/ctrlStruct.h"
#include "../CAN/CAN.h"


typedef struct ctrlStruct ctrlStruct;

typedef struct ctrlOut
{
    int r_cmd; ///< Command [-] of the right wheel : bounded in [-35 ; 35]
    int l_cmd; ///< Command [-] of the left wheel : bounded in [-35 ; 35]

    CAN can; ///< class to control motors with CAN bus

} ctrlOut;

void ctrlOut_init(ctrlOut *outputs);
void send_commands(ctrlStruct *cvs);

#endif
