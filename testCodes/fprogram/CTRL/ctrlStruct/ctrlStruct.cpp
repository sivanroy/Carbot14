/*!
 * \file ctrlStruct.cpp
 * \brief Controller main structure
 */

#include "ctrlStruct.h"
#include "../ctrl_IO/ctrlIn.h"
#include "../ctrl_IO/ctrlOut.h"
#include "../lowLevelCtrl/lowLevelCtrl.h"
#include "../myPosition/myPosition.h"
#include "../midLevelCtrlPF/midLevelCtrlPF.h"


ctrlStruct* cvs_init()
{
    ctrlStruct *cvs;
    cvs = (ctrlStruct*) malloc(sizeof(ctrlStruct));

    cvs->inputs = (ctrlIn*) malloc(sizeof(ctrlIn));
    ctrlIn_init(cvs->inputs);

    cvs->outputs = (ctrlOut*) malloc(sizeof(ctrlOut));
    ctrlOut_init(cvs->outputs);

    cvs->llc = (lowLevelCtrl*) malloc(sizeof(lowLevelCtrl));
    llc_init(cvs->llc);

    cvs->mp = (myPosition*) malloc(sizeof(myPosition));
    mp_init(cvs->mp);

    cvs->mlcPF = (midLevelCtrlPF*) malloc(sizeof(midLevelCtrlPF));
    mlcPF_init(cvs->mlcPF);

    return cvs;
}

void cvs_free(ctrlStruct *cvs)
{
    free(cvs->inputs);
    free(cvs->outputs);
    free(cvs->llc);
    free(cvs->mp);
    free(cvs->mlcPF);

    free(cvs);
}