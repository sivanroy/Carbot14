/*!
 * \file ctrlStruct.h
 * \brief Controller main structure
 */

#ifndef CARBOT14_CTRLSTRUCT_H
#define CARBOT14_CTRLSTRUCT_H

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <math.h>

#include "../ctrl_IO/ctrlIn.h"
#include "../ctrl_IO/ctrlOut.h"
#include "../lowLevelCtrl/lowLevelCtrl.h"
#include "../myPosition/myPosition.h"
#include "../midLevelCtrlPF/midLevelCtrlPF.h"
#include "../rplidar_sdk/rplStruct/rplStruct.h"
#include "../oppPosition/oppPosition.h"


typedef struct ctrlStruct ctrlStruct;
typedef struct ctrlIn ctrlIn;
typedef struct ctrlOut ctrlOut;
typedef struct lowLevelCtrl lowLevelCtrl;
typedef struct myPosition myPosition;
typedef struct midLevelCtrlPF midLevelCtrlPF;
typedef struct rplStruct rplStruct;
typedef struct oppPosition oppPosition;

typedef struct ctrlStruct
{
    ctrlIn *inputs; ///< controller inputs
    ctrlOut *outputs; ///< controller outputs

    lowLevelCtrl *llc; ///< low-level controller
    myPosition *mp; ///< localization of the robot
    midLevelCtrlPF *mlcPF; ///< middle-level controller of the Potential Field method

    rplStruct *rpl;
    oppPosition *op;

    FILE *llc_data;
    FILE *rpl_data;
    FILE *op_data;

} ctrlStruct;

ctrlStruct* cvs_init();
void cvs_free(ctrlStruct *cvs);

#endif
