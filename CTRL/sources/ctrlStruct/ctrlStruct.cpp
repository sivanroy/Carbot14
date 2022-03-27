/*!
 * \file ctrlStruct.cpp
 * \brief Controller main structure
 */

#include "ctrlStruct.h"


ctrlStruct* cvs_init()
{
    printf("cvs init : start\n");
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

    cvs->hlcPF = (highLevelCtrlPF*) malloc(sizeof(highLevelCtrlPF));
    hlcPF_init(cvs->hlcPF);

    cvs->obs = (obstacles*) malloc(sizeof(obstacles));
    obs_init(cvs->obs);    

    cvs->pshed = (pushShed*) malloc(sizeof(pushShed));
    pushShed_init(cvs->pshed);

    cvs->rpl = (rplStruct*) malloc(sizeof(rplStruct));
    rpl_init(cvs->rpl);

    //cvs->op = (oppPosition*) malloc(sizeof(oppPosition));
    //op_init(cvs->op);

    cvs->teensy = (teensyStruct*) malloc(sizeof(teensyStruct));
    teensy_init(cvs->teensy);

    // txt files
    cvs->llc_data = fopen("llc_data.txt", "w");
    if (cvs->llc_data == NULL) printf("Enable to open file llc_data.txt\n");

    return cvs;
}

void cvs_free(ctrlStruct *cvs)
{
    can_free(cvs);
    //mutex_destroy(cvs);

    free(cvs->inputs);
    free(cvs->outputs);
    free(cvs->llc);
    free(cvs->mp);
    free(cvs->mlcPF);
    free(cvs->rpl);
    free(cvs->hlcPF);
    free(cvs->obs);
    //free(cvs->op);
    free(cvs->teensy);

    fclose(cvs->llc_data);

    free(cvs);
}