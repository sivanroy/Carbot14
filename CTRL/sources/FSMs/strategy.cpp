#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <cmath> //for sqrt

#include "strategy.h"

//enum {S0_sp,SelectGoal,RunInt,Run,Ok_sp,NotOk_sp,GoHome,releaseT,Recalibrate};
enum {S0_s,select_action_s,actionPushShed_s,poseStatuette_s    ,actionPutPallet1_s,actionPutPallet2_s,actionPuPallet3_s,
                actionStatuette_s,goHome_s};

void strategy_FSM_init(strategy_FSM *stratFSM) {
    stratFSM->status = S0_s;
    stratFSM->output = 0;
    stratFSM->timeTillBreak = 15;
    stratFSM->timeToComeBack = 80;//1.30 min au total

    int s = 2;
    stratFSM->s = s;
    double timingi[s] = {30,100};
    int actionsi[s] = {actionPushShed_s,poseStatuette_s,goHome_s}; 
    for (int i = 0; i<s; i++){
            stratFSM->actions[i] = actionsi[i];
            stratFSM->timing[i]  = timingi[i];
    }
    stratFSM->pt = 0;
    printf("strategy initialized\n");
    
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
            stratFSM->status = select_action_s;
            printf("goto act\n");
            break;
        }
        case select_action_s:
            //if () si raté -> échanger l'ordre pour refaire
            if(stratFSM->pt == stratFSM->s){
                stratFSM->output = 1;
                break;
            }
            stratFSM->status = stratFSM->actions[stratFSM->pt];
            stratFSM->pt ++;
            setChrono(cvs,30,1);
            break;

        case actionPushShed_s:{
            saShed_loop(cvs);
            if(cvs->saShed->output | checkChrono(cvs,1)){
                stratFSM->status = select_action_s;
            }
            break;
        }

        case poseStatuette_s{
            poseStat_loop(cvs);
            if(cvs->poseStat | checkChrono(cvs,1)){
                stratFSM->status = select_action_s;
            }
            break;
        }

        case goHome_s:
            goHome_loop(cvs);
            if(cvs->ghome->output){
                stratFSM->status = select_action_s;
            }
            break;

        default:
            printf("probleme defautl value in FSM\n");

    }
}
