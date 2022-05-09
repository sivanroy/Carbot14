#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <cmath> //for sqrt

#include "strategy.h"

//enum {S0_sp,SelectGoal,RunInt,Run,Ok_sp,NotOk_sp,GoHome,releaseT,Recalibrate};
enum {S0_s,select_action_s,actionPushShed_s,distribution1_s,distribution2_s,poseStatuette_s,posePallet1_s,posePallet2_s,excavation_squares_s,goHome_s,goHome_shed_s};

void strategy_FSM_init(strategy_FSM *stratFSM) {
    stratFSM->status = S0_s;
    stratFSM->output = 0;
    stratFSM->timeToComeBack = 100;//1.30 min au total
    stratFSM->shed = 1;
    int s = 8;
    stratFSM->s = s;
    double maxtimingi[s] = {35,50,60,85,90,95,0,100};
    //double maxtimingi[s] = {0,0,0,0,0,0,0,100};
    double maxt = 0;
    int actionsi[s] = {actionPushShed_s,distribution1_s,poseStatuette_s,posePallet1_s,
        distribution2_s,posePallet2_s,excavation_squares_s,goHome_s}; 
    //int actionsi[s] = {distribution1_s,posePallet1_s,distribution2_s,posePallet2_s};
    for (int i = 0; i<s; i++){
            stratFSM->actions[i] = actionsi[i];
            //stratFSM->timing[i]  = timingi[i];
            stratFSM->maxTiming[i] = maxtimingi[i];
    }
    stratFSM->pt = 0;
    printf("strategy initialized\n");   
}

void checkIfEnd(ctrlStruct *cvs){
    if (cvs->inputs->t >= cvs->stratFSM->timeToComeBack) cvs->stratFSM->status = goHome_s;
}

void changeSettings(ctrlStruct *cvs){
    get_d2r_data(cvs);
    if(cvs->inputs->option1){

    }else if(cvs->inputs->option2){

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
            stratFSM->maxt = stratFSM->maxTiming[stratFSM->pt];
            printf("action selected : %d\n", stratFSM->actions[stratFSM->pt]);
            //setChrono(cvs,stratFSM->timing[stratFSM->pt],1);
            stratFSM->pt ++;
            checkIfEnd(cvs);
            if(stratFSM->status == poseStatuette_s & !cvs->saShed->gotStat){
                stratFSM->status = select_action_s;
            }
            break;

        case actionPushShed_s:{
            saShed_loop(cvs);
            if(cvs->poseStat->output == -1){
                printf("failed actionPushShed_s /: !\n");
                stratFSM->status = select_action_s;
                teensy_send(cvs,"B");
            } else if( stratFSM->maxt < inputs->t){
                printf("failed checkChrono pushshed\n");
                stratFSM->status = select_action_s;
                teensy_send(cvs,"B");
            }else if(cvs->saShed->output){
                stratFSM->status = select_action_s;
            }
            checkIfEnd(cvs);
            break;
        }

        case distribution1_s:{
            distr_loop(cvs);
            if(cvs->distr->output == -1){
                printf("failed poseStat /: !\n");
                stratFSM->status = select_action_s;
            } if(cvs->distr->output | stratFSM->maxt < inputs->t) {
                stratFSM->status = select_action_s;
            }
            checkIfEnd(cvs);
            break;
        }


        case distribution2_s:{
            distr_loop(cvs,1);
            if(cvs->distr->output == -1){
                printf("failed poseStat /: !\n");
                stratFSM->status = select_action_s;
            } if(cvs->distr->output | stratFSM->maxt < inputs->t)  {
                stratFSM->status = select_action_s;
            }
            checkIfEnd(cvs);
            break;
        }

        case poseStatuette_s:{
            poseStat_loop(cvs);
            if(cvs->poseStat->output == -1){
                printf("failed poseStat /: !\n");
                stratFSM->status = select_action_s;
            } if(cvs->poseStat->output | stratFSM->maxt < inputs->t)  {
                stratFSM->status = select_action_s;
            }
            checkIfEnd(cvs);
            break;
        }

        case posePallet1_s:{
            pPallets_loop(cvs);
            if(cvs->pPallets->output == -1){
                printf("failed poseStat /: !\n");
                stratFSM->status = select_action_s;
            } if(cvs->pPallets->output | stratFSM->maxt < inputs->t) {
                stratFSM->status = select_action_s;
                if(stratFSM->maxt < inputs->t) teensy_send(cvs,"I");
            }
            checkIfEnd(cvs);
            break;
        }

        case posePallet2_s:{
            pPallets_loop(cvs,1);
            if(cvs->pPallets->output == -1){
                printf("failed poseStat /: !\n");
                stratFSM->status = select_action_s;
            } if(cvs->pPallets->output | stratFSM->maxt < inputs->t)  {
                stratFSM->status = select_action_s;
                if(stratFSM->maxt < inputs->t) teensy_send(cvs,"I");
            }
            checkIfEnd(cvs);
            break;
        }

        case excavation_squares_s:{
            excSq_loop(cvs);
            if(cvs->excSq->output == -1){
                printf("failed excSquares /: !\n");
                stratFSM->status = select_action_s;
            } else if(stratFSM->maxt < inputs->t){
                printf("failed checkChrono excavation_squares_s\n");
                stratFSM->status = select_action_s;
            } else if(cvs->excSq->output){
                printf("ended with succes\n");
                stratFSM->status = select_action_s;
            }
            checkIfEnd(cvs);
            break;
        }

        case goHome_s:
            goHome_loop(cvs,stratFSM->shed);
            if(cvs->ghome->output){
                stratFSM->status = select_action_s;
            } 
            checkIfEnd(cvs);
            break;

        default:
            printf("probleme default value in FSM strat\n");
            checkIfEnd(cvs);
            break;

    }
}
