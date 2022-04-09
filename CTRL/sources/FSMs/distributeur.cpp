#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <cmath> //for sqrt

#include "distributeur.h"
#include "FSMs_utils.h"



enum {S0_di,DpmtHLCPF1_di,OpenDis_di,DpmtMLC_di,GetSamples_di,DpmtHLCPFOut_di};

void distr_init(distributeurs *distr){
    distr->status = S0_di;
    distr->output = 0;
    distr->go = 0;

    int s = 3; //2.28 ;; 1.51
    double x_goalsI[s] = {2.5,2.8,1};
    double y_goalsI[s] = {0.75,.75,1};
    double thetasI[s] = {-M_PI,-10,-10}; //s
    double forwardI[s] = {-1,0,-1};
    for (int i=0; i<s;i++) {
    	distr->x_goals[i] = x_goalsI[i];
    	distr->y_goals[i] = y_goalsI[i];
        distr->thetas[i] = thetasI[i];
        distr->forward[i] = forwardI[i];
    }
    printf("sashed initialized\n");
}

void distr_launch(ctrlStruct *cvs){
	cvs->distr->go = 1;
	cvs->distr->status = S0_ps;
    cvs->distr->output=0;
}


void distr_loop(ctrlStruct *cvs){
	distributeurs *distr = cvs->distr;
    myPosition *mp = cvs->mp;
    ctrlIn  *inputs = cvs->inputs;
    highLevelCtrlPF *hlcPF = cvs->hlcPF;
    midLevelCtrlPF *mlcPF = cvs->mlcPF;
    midLevelCtrl *mlc = cvs->mlc;
    teensyStruct *teensy = cvs->teensy;

    double x = mp->x; double y = mp->y;

    switch(distr->status){
        case S0_di:
        	if(distr->go){
        		distr->status = DpmtHLCPF1_di;
                set_goal(cvs,distr->x_goals[0],distr->y_goals[0],distr->thetas[0]);
        		printf("go to dpHLCPF1\n");
        		distr->go = 0;
                distr->output = 0;
        	}
            break;

        case DpmtHLCPF1_di:{
    		sendFromHLCPF(cvs,cvs->distr->forward[0]);
        	if(hlcPF->output){
                printf("goto opendis\n");
        		distr->status = OpenDis_di;
        	}
        	break;
        }

        case OpenDis_di: {
            teensy_send(cvs, "L");
            inputs->t = inputs->t + 2;
            distr->status = DpmtMLC_di;
            printf("go to dpmtmlc\n");
            break;
        }

        case DpmtMLC_di:{
            sendFromMLC(cvs,distr->x_goals[1],distr->y_goals[1],cvs->distr->forward[1]);
            if(mlc->reach_goal){
                distr->status = GetSamples_di;
                set_goal(cvs,distr->x_goals[2],distr->y_goals[2]);
                printf("go to GetSamples_di\n");
            }
            break;
        }

        case GetSamples_di:{
            //sendFromHLCPF(cvs,cvs->distr->forward[1]);
            if(hlcPF->output){
                distr->status = DpmtHLCPFOut_di;
                printf("go to Dpmt2_ps\n");
            }
            break;
        }

        case DpmtHLCPFOut_di: {
            distr->status = S0_di;
            distr->output = 1;
            break;
        }

        default:
            printf("probleme defautl value in FSM\n");
    }

}