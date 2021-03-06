/*
----------------------------
Welcome to the ctrlStruct.h
----------------------------
Controller's main structure containing all the other structures as well
as files for extracting data
-----------------------------
*/
#include "ctrlStruct.h"


ctrlStruct* cvs_init()
{
    /*!  Structures  */
    printf("cvs init : start\n");
    ctrlStruct *cvs;
    cvs = (ctrlStruct*) malloc(sizeof(ctrlStruct));

    cvs->teensy = (teensyStruct*) malloc(sizeof(teensyStruct));
    teensy_init(cvs->teensy);
    usleep(1000);
    printf("ici\n");


    cvs->rpl = (rplStruct*) malloc(sizeof(rplStruct));
    rpl_init(cvs->rpl);
    usleep(1000);
    printf("la\n");

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


    cvs->op = (oppPosition*) malloc(sizeof(oppPosition));
    op_init(cvs->op);

    cvs->mt = (mThreadsStruct*) malloc(sizeof(mThreadsStruct));
    mt_init(cvs->mt);

    cvs->rec = (reCalibStruct*) malloc(sizeof(reCalibStruct));
    rec_init(cvs->rec);

    cvs->od = (objDetection*) malloc(sizeof(objDetection));
    od_init(cvs->od);



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

    cvs->poseStat = (poseStatuette*) malloc(sizeof(poseStatuette));
    poseStat_init(cvs->poseStat);

    cvs->lpf = (lowPassFilter *) malloc(sizeof(lowPassFilter));
    lpf_init(cvs->lpf);

    cvs->chro = (Chrono *) malloc(sizeof(Chrono));
    init_chrono(cvs->chro);

    cvs->checkb = (checkBlocked *) malloc(sizeof(checkBlocked));
    init_checkBlocked(cvs->checkb);

    cvs->ghome = (goHome *) malloc(sizeof(goHome));
    goHome_init(cvs->ghome);

    /*!  txt files  */
    cvs->llc_data = fopen("llc_data.txt", "w");
    if (cvs->llc_data == NULL) printf("Enable to open file llc_data.txt\n");

    cvs->llc_data2 = fopen("llc_data2.txt", "w");
    if (cvs->llc_data2 == NULL) printf("Enable to open file llc_data2.txt\n");

    cvs->lpf_data = fopen("lpf_data.txt", "w");
    if (cvs->lpf_data == NULL) printf("Enable to open file lpf_data.txt\n");

    cvs->mlc_data = fopen("mlc_data.txt", "w");
    if (cvs->mlc_data == NULL) printf("Enable to open file mlc_data.txt\n");

    cvs->mp_data = fopen("mp_data.txt", "w");
    if (cvs->mp_data == NULL) printf("Enable to open file mp_data.txt\n");

    cvs->rpl_data = fopen("rpl_data.txt", "w");
    if (cvs->rpl_data == NULL) printf("Enable to open file rpl_data.txt\n");

    cvs->op_data = fopen("op_data.txt", "w");
    if (cvs->op_data == NULL) printf("Enable to open file op_data.txt\n");

    cvs->rec_data = fopen("rec_data.txt", "w");
    if (cvs->rec_data == NULL) printf("Enable to open file rec_data.txt\n");

    cvs->od_data = fopen("od_data.txt", "w");
    if (cvs->od_data == NULL) printf("Enable to open file od_data.txt\n");

    cvs->icp1_data = fopen("icp1_data.txt", "w");
    if (cvs->icp1_data == NULL) printf("Enable to open file icp1_data.txt\n");

    cvs->icp2_data = fopen("icp2_data.txt", "w");
    if (cvs->icp2_data == NULL) printf("Enable to open file icp2_data.txt\n");

    cvs->icp3_data = fopen("icp3_data.txt", "w");
    if (cvs->icp3_data == NULL) printf("Enable to open file icp3_data.txt\n");

    cvs->tau_data = fopen("tau_data.txt", "w");
    if (cvs->tau_data == NULL) printf("Enable to open file tau_data.txt\n");

    cvs->lidar_caract_data = fopen("lidar_caract_data.txt", "w");
    if (cvs->lidar_caract_data == NULL) printf("Enable to open file lidar_caract_data.txt\n");

    cvs->caract_odo_data = fopen("caract_odo_data.txt", "w");
    if (cvs->caract_odo_data == NULL) printf("Enable to open file caract_odo_data.txt\n");

    cvs->caract_odo_data2 = fopen("caract_odo_data2.txt", "w");
    if (cvs->caract_odo_data2 == NULL) printf("Enable to open file caract_odo_data2.txt\n");

    cvs->timing_data = fopen("timing_data.txt","w");
    if(cvs->timing_data == NULL) printf("Enab to open timing_data\n");

    return cvs;
}

void cvs_free(ctrlStruct *cvs)
{
    can_free(cvs);
    mutex_destroy(cvs);
    rpl_stop(cvs);

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
    free(cvs->od);
    free(cvs->teensy);
    free(cvs->stratFSM);
    free(cvs->saShed);
    free(cvs->pPallets);
    free(cvs->excSq);
    free(cvs->distr);
    free(cvs->poseStat);
    free(cvs->lpf);
    free(cvs->chro);
    free(cvs->checkb);
    free(cvs->ghome);

    fclose(cvs->llc_data);
    fclose(cvs->llc_data2);
    fclose(cvs->lpf_data);
    fclose(cvs->mlc_data);
    fclose(cvs->mp_data);
    fclose(cvs->rpl_data);
    fclose(cvs->op_data);
    fclose(cvs->rec_data);
    fclose(cvs->od_data);
    fclose(cvs->icp1_data);
    fclose(cvs->icp2_data);
    fclose(cvs->icp3_data);
    fclose(cvs->tau_data);
    fclose(cvs->lidar_caract_data);
    fclose(cvs->caract_odo_data);
    fclose(cvs->caract_odo_data2);
    fclose(cvs->timing_data);

    free(cvs);
}