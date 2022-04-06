#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <cmath> //for sqrt

#include "strategy.h"

//enum {S0_sp,SelectGoal,RunInt,Run,Ok_sp,NotOk_sp,GoHome,releaseT,Recalibrate};
enum {S0_s,select_action_s,actionPushShed_s,actionPutPallet1_s,actionPutPallet2_s,actionPuPallet3_s,
                actionStatuette_s,goHome_s};

void strategy_FSM_init(strategy_FSM *stratFSM) {

    stratFSM->status = S0_s;
    stratFSM->output = 0;
    stratFSM->timeTillBreak = 15;
    stratFSM->timeToComeBack = 75;//1.30 min au total

    int s = 2;
    stratFSM->s = s;
    int actionsi[s] = {actionPushShed_s,goHome_s}; 
    for (int i = 0; i<s; i++){
            stratFSM->actions[i] = actionsi[i];
    }
    
}


void strategy_loop(ctrlStruct *cvs){
    strategy_FSM *stratFSM = cvs->stratFSM;
    myPosition *mp = cvs->mp;

    ctrlIn  *inputs = cvs->inputs;
    ctrlOut *outputs= cvs->outputs;
    midLevelCtrl * mlc = cvs->mlc;
    midLevelCtrlPF *mlcPF = cvs->mlcPF;
    highLevelCtrlPF *hlcPF = cvs->hlcPF;

    //cvs->s actions
    switch(stratFSM->status){
        case S0_s:{
            stratFSM->output=0;
            stratFSM->status = actionPushShed_s;
            printf("goto act\n");
            break;
        }
        case select_action_s:
            break;
        case actionPushShed_s:
            break;
        case goHome_s:
            break;

        default:
            printf("probleme defautl value in FSM\n");

    }
}
