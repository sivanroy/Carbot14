#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <cmath> //for sqrt

#include "namespace_ctrl.h"
#include "pallet_FSM_gr5.h" // adapt it with your headers
#include "myPosition_gr5.h"
#include "highLevelCtrlPF_gr5.h"
#include "CtrlStruct_gr5.h"

#include "user_realtime.h"

NAMESPACE_INIT(ctrlGr5);


//enum States_pallet{S0,Dpmt,Wait,Ok,NotOk};

void pallet_init(pallet_FSM *palFSM) {
    palFSM->status = S0_p;
    palFSM->output = 0;
    palFSM->beginWait = -1;
    palFSM->beginWait2 = -1;
    palFSM->b2 = 0;
    palFSM->go = 0; //Need to put to 1 if go
    palFSM->goal[0] = 0; palFSM->goal[1] = 0;
    palFSM->maxTimeTillBreak = 5;
    palFSM->prev_target = 0;
}

void pallet_set_goal(CtrlStruct *cvs, double goal[2]){
    pallet_FSM *palFSM = cvs->palFSM;
    palFSM->goal[0] = goal[0];
    palFSM->goal[1] = goal[1];
    palFSM->go = 1;
}

void pallet_loop(CtrlStruct *cvs){
    pallet_FSM *palFSM = cvs->palFSM;

    myPosition *mp = cvs->mp;
    CtrlIn  *inputs = cvs->inputs;
    highLevelCtrlPF *hlcPF;

    double error = 0.1;
    double wait2 = 3;

    double x = mp->x; double y = mp->y;
    double xgoal = palFSM->goal[0]; double ygoal=palFSM->goal[1];

    //maybe smthg better ...??? from hlc???
    double xdiff = xgoal-x;
    double ydiff = ygoal-y;
    double d = sqrt(xdiff*xdiff+ydiff*ydiff);
    set_plot(d,"d-dg");
    set_plot(cvs->inputs->target_detected,"target_detected");
    switch(palFSM->status){
        case S0_p:
            if(palFSM->go){
                palFSM->output=0;
                palFSM->go=0;
                palFSM->b2 = 0;
                palFSM->status = Dpmt;
                palFSM->beginWait = inputs->t;
                palFSM->prev_target = inputs->nb_targets;
                printf("goto dpmt\n");
            }
            break;

        case Dpmt : {
            double dx = 0.02;

            if(inputs->t - palFSM->beginWait  > palFSM->maxTimeTillBreak){
                palFSM->status = NotOk;
                printf("maxTimeTillBreak\n");
            }

            if(mp->x <= 0.4+dx | mp->x >=2.6-dx) {
                //in house, cannot take this targets
                if (mp->y <=1.6 + dx & mp->y >= 1-dx){
                    //printf("in house\n");
                    break;
                }
            }
            if (inputs->target_detected & d<=.2){
                printf("detected\n");
                palFSM->status = Detected;
            }
            break;
        }

        case Detected:
            printf("detected status -> goto wait\n");
            palFSM->beginWait = inputs->t;
            palFSM->status = Wait;
            break;


        case Wait:
            if(palFSM->prev_target < inputs->nb_targets) {
                palFSM->status = Ok;
                printf("got it!\n");
            }
            if(inputs->t - palFSM->beginWait > 5){
                printf("pb while waiting /: \n");
                palFSM->status = NotOk;
            }
            break;

        case Ok:
            palFSM->output = 1;
            palFSM->go=0;
            palFSM->status = S0_p;
            printf("ok\n");
            break;

        case NotOk:
            palFSM->output =-1;
            palFSM->status = S0_p;
            palFSM->go=0;
            printf("notok\n");
            break;
        default:
            printf("probleme defautl value in FSM\n");
    }

}


NAMESPACE_CLOSE();
