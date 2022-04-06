#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <cmath> //for sqrt

#include "statuetteAndShed.h"
#include "FSMs_utils.h"




enum {S0_sas,Dpmt1_sas,servoShedOut_sas,Dpmt2_sas,Dpmt3_sas};

void saShed_init(statAndShed *saShed) {
    saShed->status = S0_ps;
    saShed->output = 0;
    saShed->go = 0;

    int s = 1; //2.28 ;; 1.51
    double x_goalsI[s] = {2.41,1,1};
    double y_goalsI[s] = {0.21,1,1};
    double thetasI[s] = {-0.77,-10,-10}; //s
    double forwardI[s] = {1,-1,-1};
    for (int i=0; i<s;i++) {
    	saShed->x_goals[i] = x_goalsI[i];
    	saShed->y_goals[i] = y_goalsI[i];
        saShed->thetas[i] = thetasI[i];
        saShed->forward[i] = forwardI[i];
    }
}

void saShed_launch(ctrlStruct *cvs){
	cvs->saShed->go = 1;
	cvs->saShed->status = S0_ps;
}


void saShed_loop(ctrlStruct *cvs){
	statAndShed *saShed = cvs->saShed;
    myPosition *mp = cvs->mp;
    ctrlIn  *inputs = cvs->inputs;
    highLevelCtrlPF *hlcPF = cvs->hlcPF;
    midLevelCtrlPF *mlcPF = cvs->mlcPF;
    midLevelCtrl *mlc = cvs->mlc;
    teensyStruct *teensy = cvs->teensy;

    double x = mp->x; double y = mp->y;

    switch(saShed->status){
        case S0_sas:
        	if(saShed->go){
        		saShed->status = Dpmt1_sas;
                set_goal(cvs,saShed->x_goals[0],saShed->y_goals[0]);
        		printf("go to dp1\n");
        		saShed->go = 0;
        	}
            break;

        case Dpmt1_sas:{
    		sendFromHLCPF(cvs,cvs->saShed->forward[0]);
        	if(hlcPF->output){
        		saShed->status = servoShedOut_sas;
        	}
        	break;
        }

        case servoShedOut_sas: {
            teensy_send(cvs, "A");
            inputs->t = inputs->t + 2;
            usleep(2000000);
            saShed->status = Dpmt2_sas;
            set_goal(cvs,saShed->x_goals[1],saShed->y_goals[1]);
            printf("go to Dpmt2_ps\n");
            break;
        }

        case Dpmt2_sas:{
            sendFromHLCPF(cvs,cvs->saShed->forward[1]);
            if(hlcPF->output){
                saShed->status = Dpmt3_sas;
                set_goal(cvs,saShed->x_goals[2],saShed->y_goals[2]);
                printf("go to Dpmt2_ps\n");
            }
            break;
        }

        case Dpmt3_sas:{
            sendFromHLCPF(cvs,cvs->saShed->forward[1]);
            if(hlcPF->output){
                saShed->status = S0_sas;
                //set_goal(cvs,saShed->x_goals[6],saShed->y_goals[6]);
                printf("go to Dpmt2_ps\n");
            }
            break;
        }
        default:
            printf("probleme defautl value in FSM\n");
    }

}