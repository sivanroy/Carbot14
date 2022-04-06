#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <cmath> //for sqrt

#include "distributeur.h"
#include "FSMs_utils.h"



enum {S0_di,Dpmt1_di,servoShedOut_di,Dpmt2_di,Dpmt3_di};

void distr_init(distributeurs *distr){
    distr->status = S0_di;
    distr->output = 0;
    distr->go = 0;

    int s = 3; //2.28 ;; 1.51
    double x_goalsI[s] = {2.41,1,1};
    double y_goalsI[s] = {0.21,1,1};
    double thetasI[s] = {-0.77,-10,-10}; //s
    double forwardI[s] = {1,-1,-1};
    for (int i=0; i<s;i++) {
    	distr->x_goals[i] = x_goalsI[i];
    	distr->y_goals[i] = y_goalsI[i];
        distr->thetas[i] = thetasI[i];
        distr->forward[i] = forwardI[i];
    }
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
        		distr->status = Dpmt1_di;
                set_goal(cvs,distr->x_goals[0],distr->y_goals[0]);
        		printf("go to dp1\n");
        		distr->go = 0;
                distr->output = 0;
        	}
            break;

        case Dpmt1_di:{
    		sendFromHLCPF(cvs,cvs->distr->forward[0]);
        	if(hlcPF->output){
        		distr->status = servoShedOut_di;
        	}
        	break;
        }

        case servoShedOut_di: {
            teensy_send(cvs, "A");
            inputs->t = inputs->t + 2;
            usleep(2000000);
            distr->status = Dpmt2_di;
            set_goal(cvs,distr->x_goals[1],distr->y_goals[1]);
            printf("go to Dpmt2_ps\n");
            break;
        }

        case Dpmt2_di:{
            sendFromHLCPF(cvs,cvs->distr->forward[1]);
            if(hlcPF->output){
                distr->status = Dpmt3_di;
                set_goal(cvs,distr->x_goals[2],distr->y_goals[2]);
                printf("go to Dpmt2_ps\n");
            }
            break;
        }

        case Dpmt3_di:{
            sendFromHLCPF(cvs,cvs->distr->forward[1]);
            if(hlcPF->output){
                distr->status = S0_di;
                distr->output = 1;
                printf("go to Dpmt2_ps\n");
            }
            break;
        }
        default:
            printf("probleme defautl value in FSM\n");
    }

}