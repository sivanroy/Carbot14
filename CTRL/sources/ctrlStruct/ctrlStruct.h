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
#include "../highLevelCtrlPF/highLevelCtrlPF.h"
#include "../highLevelCtrlPF/hlcPF_utils.h"
#include "../obstacles/obstacles.h"
#include "../FSMs/pushShed.h"
#include "../midLevelCtrl/midLevelCtrl.h"
#include "../rplidar_sdk/rplStruct/rplStruct.h"
#include "../oppPosition/oppPosition.h"
#include "../teensyStruct/teensyStruct.h"
#include "../multiThreads/multiThreads.h"
#include "../reCalibStruct/reCalibStruct.h"
#include "../FSMs/strategy.h"
#include "../FSMs/FSMs_utils.h"
#include "../FSMs/statuetteAndShed.h"
#include "../FSMs/FSMs_utils.h"


typedef struct ctrlStruct ctrlStruct;
typedef struct ctrlIn ctrlIn;
typedef struct ctrlOut ctrlOut;
typedef struct lowLevelCtrl lowLevelCtrl;
typedef struct myPosition myPosition;
typedef struct midLevelCtrlPF midLevelCtrlPF;
typedef struct rplStruct rplStruct;
typedef struct highLevelCtrlPF highLevelCtrlPF;
typedef struct obstacles obstacles;
typedef struct pushShed pushShed;
typedef struct oppPosition oppPosition;
typedef struct teensyStruct teensyStruct;
typedef struct midLevelCtrl midLevelCtrl;
typedef struct mThreadsStruct mThreadsStruct;
typedef struct reCalibStruct reCalibStruct;
typedef struct strategy_FSM strategy_FSM;
typedef struct statAndShed statAndShed;



typedef struct ctrlStruct
{
    ctrlIn *inputs; ///< controller inputs
    ctrlOut *outputs; ///< controller outputs

    lowLevelCtrl *llc; ///< low-level controller
    myPosition *mp; ///< localization of the robot
    midLevelCtrlPF *mlcPF; ///< middle-level controller of the Potential Field method
    highLevelCtrlPF *hlcPF;
    obstacles *obs;
    midLevelCtrl *mlc;

    strategy_FSM *stratFSM;
    pushShed *pshed;
    statAndShed *saShed;

    rplStruct *rpl;
    oppPosition *op;
    mThreadsStruct *mt;
    reCalibStruct *rec;

    teensyStruct *teensy;

    FILE *llc_data;
    FILE *mp_data;
    FILE *rpl_data;
    FILE *op_data;
    FILE *rec_data;

} ctrlStruct;

ctrlStruct* cvs_init();
void cvs_free(ctrlStruct *cvs);

#endif
