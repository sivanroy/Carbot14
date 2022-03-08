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


typedef struct ctrlOut
{
    int r_cmd; ///< Command [-] of the right wheel : bounded in [-35 ; 35]
    int l_cmd; ///< Command [-] of the left wheel : bounded in [-35 ; 35]

} ctrlOut;

void ctrlOut_init(ctrlOut *outputs);

#endif
