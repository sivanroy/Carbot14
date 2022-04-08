/*!
 * \file ctrlStruct.cpp
 * \brief Controller main structure
 */

#include "ctrlStruct.h"


ctrlStruct* cvs_init()
{
    /*!  Structures  */
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

    cvs->mlc = (midLevelCtrl*) malloc(sizeof(midLevelCtrl));
    init_midLevelCtrl(cvs->mlc);

    cvs->pshed = (pushShed*) malloc(sizeof(pushShed));
    pushShed_init(cvs->pshed);

    cvs->rpl = (rplStruct*) malloc(sizeof(rplStruct));
    rpl_init(cvs->rpl);

    cvs->op = (oppPosition*) malloc(sizeof(oppPosition));
    op_init(cvs->op);

    cvs->mt = (mThreadsStruct*) malloc(sizeof(mThreadsStruct));
    mt_init(cvs->mt);

    cvs->rec = (reCalibStruct*) malloc(sizeof(reCalibStruct));
    rec_init(cvs->rec);

    cvs->teensy = (teensyStruct*) malloc(sizeof(teensyStruct));
    teensy_init(cvs->teensy);

    cvs->stratFSM = (strategy_FSM *) malloc(sizeof(strategy_FSM));
    strategy_FSM_init(cvs->stratFSM);

    cvs->saShed = (statAndShed*) malloc(sizeof(statAndShed));
    saShed_init(cvs->saShed);

    cvs->pPallets = (posePallets*) malloc(sizeof(posePallets));
    pPallets_init(cvs->pPallets);

    cvs->excSq = (excSquares*) malloc(sizeof(excSquares));
    excSq_init(cvs->excSq);

    cvs->distr = (distributeurs*) malloc(sizeof(distributeurs));
    distr_init(cvs->distr);

    /*!  txt files  */
    cvs->llc_data = fopen("llc_data.txt", "w");
    if (cvs->llc_data == NULL) printf("Enable to open file llc_data.txt\n");

    cvs->mp_data = fopen("mp_data.txt", "w");
    if (cvs->mp_data == NULL) printf("Enable to open file mp_data.txt\n");

    cvs->rpl_data = fopen("rpl_data.txt", "w");
    if (cvs->rpl_data == NULL) printf("Enable to open file rpl_data.txt\n");

    cvs->op_data = fopen("op_data.txt", "w");
    if (cvs->op_data == NULL) printf("Enable to open file op_data.txt\n");

    cvs->rec_data = fopen("rec_data.txt", "w");
    if (cvs->rec_data == NULL) printf("Enable to open file rec_data.txt\n");

    cvs->tau_data = fopen("tau_data.txt", "w");
    if (cvs->tau_data == NULL) printf("Enable to open file tau_data.txt\n");


    return cvs;
}

void cvs_free(ctrlStruct *cvs)
{
    can_free(cvs);
    mutex_destroy(cvs);
    //rpl_stop(cvs);

    free(cvs->inputs);
    free(cvs->outputs);
    free(cvs->llc);
    free(cvs->mp);
    free(cvs->mlcPF);
    free(cvs->rpl);
    free(cvs->hlcPF);
    free(cvs->obs);
    free(cvs->op);
    free(cvs->mt);
    free(cvs->rec);
    free(cvs->teensy);
    free(cvs->stratFSM);
    free(cvs->saShed);
    free(cvs->pPallets);
    free(cvs->excSq);
    free(cvs->distr);

    fclose(cvs->llc_data);
    fclose(cvs->mp_data);
    fclose(cvs->rpl_data);
    fclose(cvs->op_data);
    fclose(cvs->rec_data);
    fclose(cvs->tau_data);

    free(cvs);
}