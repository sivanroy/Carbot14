#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <cmath> //for sqrt

#include "namespace_ctrl.h"
#include "pallet_FSM_gr5.h" // adapt it with your headers
#include "myPosition_gr5.h"
#include "select_Pallet_FSM_gr5.h"
#include "calibration_gr5.h"


#include "highLevelCtrlPF_gr5.h"
#include "midLevelCtrlPF_gr5.h"
#include "lowLevelCtrl_gr5.h"
#include "ctrl_io.h"
#include "midLevelCtrl_gr5.h"


#include "CtrlStruct_gr5.h"

#include "user_realtime.h"

NAMESPACE_INIT(ctrlGr5);


//enum {S0_sp,SelectGoal,RunInt,Run,Ok_sp,NotOk_sp,GoHome,releaseT,Recalibrate};


void select_pallet_FSM_init(select_pallet_FSM *spFSM) {
    spFSM->status = S0_sp;
    spFSM->go = 0;
    spFSM->output = 0;
    spFSM->nb_targets = 0;
    spFSM->beginFSM = -1;
    spFSM->timeTillBreak = 15;

    spFSM->timeToComeBack = 75;//1.30 min au total
    spFSM->goal = -1; //home

    spFSM->s = 7; int s = spFSM->s;
    double xint[7] = {.3,.3,2.7,2.7 ,   1.8 ,1.2,   1.5};
    double yint[7] = {1.7,.55,.55,1.7,  1.55,1.55,  .3};

    double xt[7] = {.15,.2,2.8,2.85 ,   1.7 ,1.3,   1.5}; //goals
    double yt[7] = {1.85,.55,.55,1.85,  1.75,1.75,  .15};

    int p[7] = {3,3,3,3,3,3,3};

    for (int i =0;i<s;i++){
        spFSM->xtargets[i] = xt[i];
        spFSM->ytargets[i] = yt[i];
        spFSM->xint[i] = xint[i];
        spFSM->yint[i] = yint[i],
        spFSM->targetsStatus[i] = 0;
        spFSM->priority[i] = p[i];
    }

}

void set_priority(CtrlStruct *cvs){
    int *p = cvs->spFSM->priority;
    if(cvs->team_id == TEAM_A){
        p[6] = 1;
        p[5] = 0;
        p[4] = 0;
        p[2] = 2;
        p[3] = 2;
    } else {
        p[6] = 1;
        p[5] = 0;
        p[4] = 0;
        p[0] = 2;
        p[1] = 2;        
    }
}

int select_action(CtrlStruct *cvs,int TA){ //tp is target points (1 or 2)

    select_pallet_FSM * spFSM = cvs->spFSM;
    myPosition *mp = cvs->mp;

    double betterd = 100;
    int betterGoal = -1;
    int betterPrior = 100;
    double d;
    double xgoal; double ygoal;
    double goalxy[2];

    for (int i = 0; i<spFSM->s; i++){
        int prior = spFSM->priority[i];
        if(spFSM->targetsStatus[i]==0){
            if (prior <= betterPrior) {
                double x = mp->x; double y =mp->y;
                xgoal = spFSM->xtargets[i];
                ygoal = spFSM->ytargets[i];
                d = sqrt((x-xgoal)*(x-xgoal)+(y-ygoal)*(y-ygoal));
                if(betterGoal==-1 or d < betterd or prior<betterPrior){
                    printf("goal %d\n",i);
                    betterd = d;
                    betterGoal = i;
                    betterPrior = prior;
                }
            }
        }
    }

    if (betterGoal != -1) {
        spFSM->goal = betterGoal;
        goalxy[0] = spFSM->xtargets[betterGoal];
        goalxy[1] = spFSM->ytargets[betterGoal];
        pallet_set_goal(cvs,goalxy);
        spFSM->status = RunInt;
        printf("betterGoal = %d\n",betterGoal );
        printf("target is %d priority\n",betterPrior);
        return 1;
    }

    return -1;
}

void select_pallet_loop(CtrlStruct *cvs){
    select_pallet_FSM * spFSM = cvs->spFSM;
    pallet_FSM *palFSM = cvs->palFSM;
    myPosition *mp = cvs->mp;
    CtrlIn  *inputs = cvs->inputs;
    CtrlOut *outputs= cvs->outputs;

    midLevelCtrl * mlc = cvs->mlc;
    midLevelCtrlPF *mlcPF = cvs->mlcPF;
    highLevelCtrlPF *hlcPF = cvs->hlcPF;

    //printf("mp->x : %f | mp->y  %f\n",mp->x,mp->y);
    double xgoal;
    double ygoal;
    double goalxy[2];
    switch(spFSM->status){
        case S0_sp:
            if(spFSM->go){
                spFSM->output=0;
                spFSM->status = SelectGoal;
                printf("goto selectGoal\n");
            }
            break;

        case SelectGoal:{
            double goalxy[2];
            spFSM->beginFSM = inputs->t;
            //have to come back because time is running
            if(inputs->t > spFSM->timeToComeBack){
                spFSM->status = GoHome;
                printf("come back\n");
                break;
            }
            //have 2 targets -> need to go home
            printf("nb targets = %d\n",inputs->nb_targets);
            if (inputs->nb_targets == 2  | spFSM->nb_targets == 2){
                spFSM->goal = -1;
                spFSM->status = GoHome;
                goalxy[0] = mp->house_xy[0];
                goalxy[1] = mp->house_xy[1];
                pallet_set_goal(cvs,goalxy);
                printf("t is home\n");
                break;
            }
            int TA = cvs->team_id == TEAM_A;
            int out = select_action(cvs,TA);
            if(out != -1) {
                break;
            } else {
                spFSM->status = GoHome;
                printf("No targets left\n");
                break;
            }
        }

        case RunInt : {
            //go to intermediate points
            if(inputs->t >spFSM->timeToComeBack){
                spFSM->status = GoHome;
                printf("come back\n");
                break;
            }
            if(inputs->t - spFSM->beginFSM > spFSM->timeTillBreak){
                printf("could not go to intermediate targets\n");
                spFSM->status = NotOk_sp;
                break;
            }
            double xg = spFSM->xint[spFSM->goal];
            double yg = spFSM->yint[spFSM->goal];

            main_pot_force(cvs,xg,yg);

            if (hlcPF->output == 1){
                printf("goal int : x %f | y %f\n",xg,yg );
                printf("arrived in RunInt\n");
                spFSM->status = Run;
                break;
            } else {
                mlcPF_out(cvs, hlcPF->v_ref, hlcPF->theta_ref);
                set_plot(mlcPF->r_sp_ref/10, "r_sp_ref/10");
                //printf("rsp_ref%f\n", mlcPF->r_sp_ref);
                set_commands(cvs, mlcPF->r_sp_ref, mlcPF->l_sp_ref);
                break;
            }

            break;
        }

        case Run:{
            //exceeded time
            if(inputs->t >spFSM->timeToComeBack){
                spFSM->status = GoHome;
                printf("come back\n");
                break;
            }
            //run FSM
            pallet_loop(cvs);

            xgoal = palFSM->goal[0];
            ygoal = palFSM->goal[1];

            set_speed_ref(cvs, xgoal, ygoal);

            //arrived at dest hlcPF
            if (mlc->reach_goal){
                //printf("goal : x %f | y %f\n",xgoal,ygoal );
                //printf("arrived in Run\n");
            } else {
                set_commands(cvs, mlc->r_sp_ref, mlc->l_sp_ref);
            }

            if (palFSM->output != 0){
                if(palFSM->output == 1) {
                    spFSM->status = Ok_sp;
                } else {
                    spFSM->status = NotOk_sp;
                }
            }
            break;
        }

        case Ok_sp:
            spFSM->targetsStatus[spFSM->goal] = 1;
            spFSM->status = SelectGoal;
            spFSM->nb_targets += 1;
            printf("ok \n");
            break;

        case NotOk_sp:
            spFSM->targetsStatus[spFSM->goal] = -1;
            spFSM->status = SelectGoal;
            printf("notok\n");
            break;

        case GoHome:{
            main_pot_force(cvs,mp->house_xy[0],mp->house_xy[1]);
            mlcPF_out(cvs, hlcPF->v_ref, hlcPF->theta_ref);
            set_commands(cvs, mlcPF->r_sp_ref, mlcPF->l_sp_ref);

            if(cvs->hlcPF->output == 1 | cvs->hlcPF->d < 0.05) {
                spFSM->status = releaseT;
                printf("go to release status\n");
            }
            break;
        }

        case releaseT:
            outputs->flag_release = 1;
            spFSM->nb_targets = 0;
            spFSM->status = Recalibrate;
            calib_re_init(cvs);
            if(inputs->t > spFSM->timeToComeBack){
                spFSM->go = 0;
                spFSM->status = S0_sp;
                printf("end of FSM\n");
            }
            printf("Recalibrate\n");
            break;

        case Recalibrate: {
            double dth = 0.4;
            double th_r = 0;
            outputs->flag_release = 0;
            if(cvs->team_id == TEAM_A){
                th_r = 3.1415;
            }
            //printf("theta : %f | th_r %f | %f | %f | %f \n", mp->th, th_r, abs(mp->th-th_r), fabs(mp->th-th_r),std::abs(mp->th-th_r));
            if( std::abs(mp->th-th_r) >= dth) {
                //printf("rotate\n");
                mlcPF_out(cvs, 0, th_r);
                set_commands(cvs, mlcPF->r_sp_ref, mlcPF->l_sp_ref);
                break;
            } else {
                calibrate(cvs);
                //printf("calbiration in process\n");
                if(cvs->calib->output != 0) {
                    if(cvs->calib->output == 1) {
                        spFSM->status = S0_sp;
                        printf("Recalibrated\n");
                    } else {
                        printf("error occured in calibration ... \n");
                        spFSM->status = S0_sp;
                    }
                }
            }
            break;
        }

        default:
            printf("probleme defautl value in FSM\n");

    }
}


NAMESPACE_CLOSE();
