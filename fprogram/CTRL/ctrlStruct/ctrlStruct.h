/*!
 * \file ctrlStruct.h
 * \brief Controller main structure
 */

#ifndef _CTRLSTRUCT_H_
#define _CTRLSTRUCT_H_

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <math.h>


typedef struct ctrlStruct
{
    ctrlIn *inputs; ///< controller inputs
    ctrlOut *outputs; ///< controller outputs

    lowLevelCtrl *llc; ///< low-level controller
    midLevelCtrlPF *mlcPF; ///< middle-level controller of the Potential Field method

} ctrlStruct;

void

#endif
