#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <cmath> //for sqrt

#include "posePallets.h"
#include "FSMs_utils.h"



enum {S0_pp,Dpmt1_pp,servoShedOut_pp,Dpmt2_pp,Dpmt3_pp};

void pPallets_init(posePallets *pPallets){
    pPallets->status = S0_ps;
    pPallets->output = 0;
    pPallets->go = 0;

    int s = 3; //2.28 ;; 1.51
    double x_goalsI[s] = {2.41,1,1};
    double y_goalsI[s] = {0.21,1,1};
    double thetasI[s] = {-0.77,-10,-10}; //s
    double forwardI[s] = {1,-1,-1};
    for (int i=0; i<s;i++) {
    	pPallets->x_goals[i] = x_goalsI[i];
    	pPallets->y_goals[i] = y_goalsI[i];
        pPallets->thetas[i] = thetasI[i];
        pPallets->forward[i] = forwardI[i];
    }
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
    
    switch(pPallets->status){
        case S0_pp:
        	if(pPallets->go){
        		pPallets->status = Dpmt1_pp;
                set_goal(cvs,pPallets->x_goals[0],pPallets->y_goals[0]);
        		printf("go to dp1\n");
        		pPallets->go = 0;
        	}
            break;

        case Dpmt1_pp:{
    		sendFromHLCPF(cvs,cvs->pPallets->forward[0]);
        	if(hlcPF->output){
        		pPallets->status = servoShedOut_pp;
        	}
        	break;
        }

        case servoShedOut_pp: {
            teensy_send(cvs, "A");
            inputs->t = inputs->t + 2;
            usleep(2000000);
            pPallets->status = Dpmt2_pp;
            set_goal(cvs,pPallets->x_goals[1],pPallets->y_goals[1]);
            printf("go to Dpmt2_ps\n");
            break;
        }

        case Dpmt2_pp:{
            sendFromHLCPF(cvs,cvs->pPallets->forward[1]);
            if(hlcPF->output){
                pPallets->status = Dpmt3_pp;
                set_goal(cvs,pPallets->x_goals[2],pPallets->y_goals[2]);
                printf("go to Dpmt2_ps\n");
            }
            break;
        }

        case Dpmt3_pp:{
            sendFromHLCPF(cvs,cvs->pPallets->forward[1]);
            if(hlcPF->output){
                pPallets->status = S0_pp;
                printf("go to Dpmt2_ps\n");
            }
            break;
        }
        default:
            printf("probleme defautl value in FSM\n");
    }

}