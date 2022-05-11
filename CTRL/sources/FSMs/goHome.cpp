#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <cmath> //for sqrt

#include "goHome.h"

//enum {S0_sp,SelectGoal,RunInt,Run,Ok_sp,NotOk_sp,GoHome,releaseT,Recalibrate};
enum {S0_gh,dp1_gh,rec_gh,dp2_gh,dpSq1_gh,dpSq2_gh,rec1_gh,dp_prec_Sq_gh};

void goHome_init(goHome *ghome) {
    ghome->status = S0_gh;
    ghome->output = 0;
    printf("gohome initialized\n");
}


void checkTime(ctrlStruct *cvs){
    int TEAM = cvs->inputs->team;
    if(cvs->inputs->t >= 98.5){
        if (TEAM) set_goal(cvs,3-.85,.4,-10); //2.32,1.51
        else set_goal(cvs,.85,.4,-10);
        cvs->ghome->status = dp1_gh;
    }
}

void goHome_loop(ctrlStruct *cvs,int shed){
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
    //85.5
    double wait = 0.6;
    //cvs->s actions
    if(shed){
        checkTime(cvs);
        switch(ghome->status){
            case S0_gh:{
                ghome->output=0;
                ghome->status = dpSq1_gh;
                printf("goto act\n");
                if (TEAM) set_goal(cvs,3-.6,.6,-10); //2.32,1.51
                else set_goal(cvs,.6,.5,-10);
                teensy_send(cvs,"Z");                    
                break;
            }

            case dpSq1_gh:{
                hlcPF->Tau_max = 15;
                cvs->mlcPF->K_orient = 5;
                cvs->mlcPF->max_th = 12;
                sendFromHLCPF(cvs,-1,1,1);
                if(hlcPF->output){
                    ghome->status = dpSq2_gh;
                    if (TEAM) set_goal(cvs,3-.855,.45,M_PI/2); //2.32,1.51
                    else set_goal(cvs,.855,.45,M_PI/2);
                    setChrono(cvs,wait);                    
                }
                break;
            }

            case dpSq2_gh:{
                hlcPF->Tau_max = 15;
                cvs->mlcPF->K_orient = 5;
                cvs->mlcPF->max_th = 12;
                sendFromHLCPF(cvs,-1,1,1);
                if(hlcPF->output){
                    ghome->status = rec1_gh;
                    if (TEAM) set_goal(cvs,3-.855,0.13,-10); //2.32,1.51
                    else set_goal(cvs,.855,0.13,-10);
                    setChrono(cvs,wait);                    
                }
                break;
            }

            case rec1_gh:{
                if (rec_static(cvs) || checkChrono(cvs)) {
                    printf("rec_gh END : go to dp2_gh\n");
                    ghome->status = dp_prec_Sq_gh;
                }
                break;
            }

            case dp_prec_Sq_gh:{
                cvs->mlcPF->max_th = 5;
                set_param_prec(cvs);
                sendFromHLCPF(cvs,0,1);
                if(hlcPF->output){
                    printf("ended gh in t = %d\n\n----------\n",cvs->inputs->t);
                    arduino_send(cvs,"A");
                    if (TEAM) set_goal(cvs,3-.85,.4,-10); //2.32,1.51
                    else set_goal(cvs,.85,.4,-10);
                    ghome->status = dp1_gh;
                }
                break;
            }

            case dp1_gh:{
                sendFromHLCPF(cvs,-1,1,1);
                if(hlcPF->output){
                    ghome->status = rec_gh;
                    if (TEAM) set_goal(cvs,3-.85,.5,M_PI/2); //2.32,1.51
                    else set_goal(cvs,.85,.5,M_PI/2);
                    arduino_send(cvs,"K");
                    //teensy_send(cvs,"I");
                    setChrono(cvs,wait);
                    ghome->output =1;
                }
                break;
            }
            case rec_gh:{
                if (rec_static(cvs) || checkChrono(cvs)) {
                    printf("rec_gh END : go to dp2_gh\n");
                    ghome->status = dp2_gh;
                }
                break;
            }
            case dp2_gh:{
                set_param_prec(cvs);
                sendFromHLCPF(cvs,-1,1);
                if(hlcPF->output){
                    cvs->ghome->output = 1;
                    printf("ended gh in t = %d\n\n----------\n",cvs->inputs->t);
                }
                break;
            }

            default:
                printf("probleme defautl value in FSM\n");

        }
    }else {
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
                    ghome->status = rec_gh;
                    if (TEAM) set_goal(cvs,2.65,1.3,0); //2.32,1.51
                    else set_goal(cvs,0.35,1.3,M_PI);
                    arduino_send(cvs,"K");
                    setChrono(cvs,wait);
                    cvs->ghome->output = 1;
                }
                break;
            }
            case rec_gh:{
                if (rec_static(cvs) || checkChrono(cvs)) {
                    printf("rec_gh END : go to dp2_gh\n");
                    ghome->status = dp2_gh;
                }
                break;
            }
            case dp2_gh:{
                set_param_prec(cvs);
                sendFromHLCPF(cvs,-1,1);
                if(hlcPF->output){
                    cvs->ghome->output = 1;
                    printf("ended gh in t = %d\n\n----------\n",cvs->inputs->t);
                }
                break;
            }

            default:
                printf("probleme defautl value in FSM\n");

        }
    }
}
