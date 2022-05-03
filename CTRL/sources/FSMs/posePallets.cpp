#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <cmath> //for sqrt

#include "posePallets.h"
#include "FSMs_utils.h"



enum {S0_pp,Dpmt1_pp,rec1_pp,Dpmt1prec_pp,pose1_pp,
    Dpmt2_pp,rec2_pp,Dpmt2prec_pp,pose2_pp,
    Dpmt3_pp,rec3_pp,Dpmt3prec_pp,pose3_pp,
    Dptout_pp};

void pPallets_init(posePallets *pPallets){
    pPallets->status = S0_ps;
    pPallets->output = 0;
    pPallets->go = 0;
    printf("ppallets initialized\n");

}

void pPallets_launch(ctrlStruct *cvs){
	cvs->pPallets->go = 1;
	cvs->pPallets->status = S0_ps;
    cvs->pPallets->output = 0;
}


void pPallets_loop(ctrlStruct *cvs){
	posePallets *pPallets = cvs->pPallets;
    myPosition *mp = cvs->mp;
    ctrlIn  *inputs = cvs->inputs;
    highLevelCtrlPF *hlcPF = cvs->hlcPF;
    midLevelCtrlPF *mlcPF = cvs->mlcPF;
    midLevelCtrl *mlc = cvs->mlc;
    teensyStruct *teensy = cvs->teensy;


    int TEAM = cvs->inputs->team;
    //only if usefull
    double pos[5];
    double x, y, th;
    get_pos(cvs, pos);
    th = pos[2];
    x = pos[0];//+ hlcPF->x_shift * cos(th);
    y = pos[1];//+ hlcPF->x_shift * sin(th);
    set_param_normal(cvs);
    double dy = -0.035;
    double wait = .6;
    double waitForP = 3;
    switch(pPallets->status){
        case S0_pp:
        	pPallets->status = Dpmt1_pp;
            if (TEAM) set_goal(cvs,3-1.05,1.55,-M_PI/2);
            else set_goal(cvs,1.05,1.55,-M_PI/2);
            printf("go to dp1\n");
        	pPallets->go = 0;
        	break;

        case Dpmt1_pp:{
    		sendFromHLCPF(cvs,-1,1,1);
        	if(hlcPF->output){
        		pPallets->status = rec1_pp;
                teensy_send(cvs,"F");
                setChrono(cvs,wait,2);
                setChrono(cvs,waitForP);
                if (TEAM) set_goal(cvs,3-1.05,2-.21+dy,-M_PI/2);
                else set_goal(cvs,1.05,2-.21+dy,-M_PI/2);
        	}
        	break;
        }

        case rec1_pp:{
            if (rec_static(cvs) | checkChrono(cvs,2)) {
                printf("rec_start_es END : go to Dpmt2_es\n");
                pPallets->status = Dpmt1prec_pp;
            }
            break;
        }

        case Dpmt1prec_pp: {
            set_param_prec(cvs);
            sendFromHLCPF(cvs,0,1);
            if(hlcPF->output & checkChrono(cvs)){
                pPallets->status = pose1_pp;
                setChrono(cvs,1);
                printf("go to pose1_ps\n");
                teensy_send(cvs,"I");
                arduino_send(cvs,"6");
            }
            break;
        }

        case pose1_pp: {
            if (checkChrono(cvs)) {
                pPallets->status = Dpmt2_pp;
                if (TEAM) set_goal(cvs,3-.81,1.55,-M_PI/2);
                else set_goal(cvs,.81,1.55,-M_PI/2);
                teensy_send(cvs,"G");
                setChrono(cvs,waitForP);
            }
            break;
        }

        case Dpmt2_pp:{
            sendFromHLCPF(cvs,-1,1,1);
            if(hlcPF->output & checkChrono(cvs)){
                pPallets->status = rec2_pp;
                if (TEAM) set_goal(cvs,3-.81,2-.21+dy,-M_PI/2);
                else set_goal(cvs,.81,2-.19+dy,-M_PI/2);
            }
            break;
        }

        case rec2_pp:{
            if (rec_static(cvs) | checkChrono(cvs,2)) {
                printf("rec_start_es END : go to Dpmt2_es\n");
                pPallets->status = Dpmt2prec_pp;
            }
            break;
        }

        case Dpmt2prec_pp: {
            set_param_prec(cvs);
            sendFromHLCPF(cvs,0,1);
            if(hlcPF->output){
                pPallets->status = pose2_pp;
                setChrono(cvs,1);
                printf("go to pose2_ps\n");
                teensy_send(cvs,"I");
                arduino_send(cvs,"6");

            }
            break;
        }

        case pose2_pp: {
            if (checkChrono(cvs)) {
                pPallets->status = Dpmt3_pp;
                if (TEAM) set_goal(cvs,3-.57,1.55,-M_PI/2);
                else set_goal(cvs,.57,1.55,-M_PI/2);
                teensy_send(cvs,"H");
                setChrono(cvs,waitForP);
            }
            break;
        }

        case Dpmt3_pp:{
            sendFromHLCPF(cvs,-1,1,1);
            if(hlcPF->output&checkChrono(cvs)){
                pPallets->status = rec3_pp;
                if (TEAM) set_goal(cvs,3-.57,2-.21+dy,-M_PI/2);
                else set_goal(cvs,.57,2-.21+dy,-M_PI/2);
            }
            break;
        }

        case rec3_pp:{
            if (rec_static(cvs) | checkChrono(cvs,2)) {
                printf("rec_start_es END : go to Dpmt2_es\n");
                pPallets->status = Dpmt3prec_pp;
            }
            break;
        }

        case Dpmt3prec_pp: {
            set_param_prec(cvs);
            sendFromHLCPF(cvs,0,1);
            if(hlcPF->output){
                pPallets->status = pose3_pp;
                setChrono(cvs,1);
                printf("go to pose3_ps\n");
                teensy_send(cvs,"I");
                arduino_send(cvs,"6");
            }
            break;
        }

        case pose3_pp: {
            if (checkChrono(cvs)) {
                set_goal(cvs,1,1,M_PI/2);
                if (TEAM) set_goal(cvs,3-.57,1.5,-10);
                else set_goal(cvs,.57,1.5,-10);
                pPallets->status = Dptout_pp;
            }
            break;
        }

        case Dptout_pp: {
            sendFromHLCPF(cvs,-1);
            if(hlcPF->output){
                pPallets->output = 1;
            }
        }

        default:
            printf("probleme defautl value in FSM\n");
    }

}