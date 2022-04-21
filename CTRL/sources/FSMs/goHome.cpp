#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <cmath> //for sqrt

#include "goHome.h"

//enum {S0_sp,SelectGoal,RunInt,Run,Ok_sp,NotOk_sp,GoHome,releaseT,Recalibrate};
enum {S0_gh,dp1_gh,dp2_gh};

void goHome_init(goHome *ghome) {
    ghome->status = S0_gh;
    ghome->output = 0;
    printf("gohome initialized\n");
}


void goHome_loop(ctrlStruct *cvs){
    goHome *ghome = cvs->ghome;
    myPosition *mp = cvs->mp;
    ctrlIn  *inputs = cvs->inputs;
    ctrlOut *outputs= cvs->outputs;
    midLevelCtrl * mlc = cvs->mlc;
    midLevelCtrlPF *mlcPF = cvs->mlcPF;
    highLevelCtrlPF *hlcPF = cvs->hlcPF;

    set_param_normal(cvs);
    int TEAM = cvs->inputs->team;

    //only if usefull
    double pos[5];
    double x, y, th;
    get_pos(cvs, pos);
    th = pos[2];
    x = pos[0];//+ hlcPF->x_shift * cos(th);
    y = pos[1];//+ hlcPF->x_shift * sin(th);

    //cvs->s actions
    switch(ghome->status){
        case S0_gh:{
            ghome->output=0;
            ghome->status = dp1_gh;
            printf("goto act\n");
            if (TEAM) set_goal(cvs,2.6,1.3,0); //2.32,1.51
            else set_goal(cvs,0.4,1.3,M_PI);
            break;
        }

        case dp1_gh:{
            set_param_normal(cvs);
            sendFromHLCPF(cvs,-1);
            if(hlcPF->output){
                ghome->status = dp2_gh;
                if (TEAM) set_goal(cvs,2.75,1.3,0); //2.32,1.51
                else set_goal(cvs,0.25,1.3,M_PI);
                arduino_send(cvs,"K");
            }
            break;
        }

        case dp2_gh:{
            set_param_prec(cvs);
            sendFromHLCPF(cvs,-1,1);
            if(hlcPF->output){
                cvs->ghome->output = 1;
            }
            break;
        }

        default:
            printf("probleme defautl value in FSM\n");

    }
}
