#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <cmath> //for sqrt

#include "posePallets.h"
#include "FSMs_utils.h"



enum {S0_es,Dpmt1_es,servoShedOut_es,Dpmt2_es,Dpmt3_es};

void pPalets_init(posePallets *pPalets){
    pPalets->status = S0_ps;
    pPalets->output = 0;
    pPalets->go = 0;

    int s = 1; //2.28 ;; 1.51
    double x_goalsI[s] = {2.41,1,1};
    double y_goalsI[s] = {0.21,1,1};
    double thetasI[s] = {-0.77,-10,-10}; //s
    double forwardI[s] = {1,-1,-1};
    for (int i=0; i<s;i++) {
    	pPalets->x_goals[i] = x_goalsI[i];
    	pPalets->y_goals[i] = y_goalsI[i];
        pPalets->thetas[i] = thetasI[i];
        pPalets->forward[i] = forwardI[i];
    }
}

void pPalets_launch(ctrlStruct *cvs){
	cvs->pPalets->go = 1;
	cvs->pPalets->status = S0_ps;
    cvs->pPalets->output = 0;
}


void pPalets_loop(ctrlStruct *cvs){
	posePallets *pPalets = cvs->pPalets;
    myPosition *mp = cvs->mp;
    ctrlIn  *inputs = cvs->inputs;
    highLevelCtrlPF *hlcPF = cvs->hlcPF;
    midLevelCtrlPF *mlcPF = cvs->mlcPF;
    midLevelCtrl *mlc = cvs->mlc;
    teensyStruct *teensy = cvs->teensy;

    double x = mp->x; double y = mp->y;

    switch(pPalets->status){
        case S0_sas:
        	if(pPalets->go){
        		pPalets->status = Dpmt1_sas;
                set_goal(cvs,pPalets->x_goals[0],pPalets->y_goals[0]);
        		printf("go to dp1\n");
        		pPalets->go = 0;
        	}
            break;

        case Dpmt1_sas:{
    		sendFromHLCPF(cvs,cvs->pPalets->forward[0]);
        	if(hlcPF->output){
        		pPalets->status = servoShedOut_sas;
        	}
        	break;
        }

        case servoShedOut_sas: {
            teensy_send(cvs, "A");
            inputs->t = inputs->t + 2;
            usleep(2000000);
            pPalets->status = Dpmt2_sas;
            set_goal(cvs,pPalets->x_goals[1],pPalets->y_goals[1]);
            printf("go to Dpmt2_ps\n");
            break;
        }

        case Dpmt2_sas:{
            sendFromHLCPF(cvs,cvs->pPalets->forward[1]);
            if(hlcPF->output){
                pPalets->status = Dpmt3_sas;
                set_goal(cvs,pPalets->x_goals[2],pPalets->y_goals[2]);
                printf("go to Dpmt2_ps\n");
            }
            break;
        }

        case Dpmt3_sas:{
            sendFromHLCPF(cvs,cvs->pPalets->forward[1]);
            if(hlcPF->output){
                pPalets->status = S0_sas;
                printf("go to Dpmt2_ps\n");
            }
            break;
        }
        default:
            printf("probleme defautl value in FSM\n");
    }

}