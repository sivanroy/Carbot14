#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <cmath> //for sqrt

#include "statuetteAndShed.h"
#include "FSMs_utils.h"



enum {S0_sas,Dpmt1_sas,servoShedOut_sas,Dpmt2_sas,Dpmt3_sas,Dpmt4_sas,Dpmt5_sas, servoShedIn_sas};

void saShed_init(statAndShed *saShed) {
    saShed->status = S0_ps;
    saShed->output = 0;
    saShed->go = 0;

    int s = 5; //2.2 ;; 1.6 ;; -2.5
    double x_goalsI[s] = {2.44,2.33, 2.1, 2.1,2.5};
    double y_goalsI[s] = {1.55,1.5, 1.35, 1.2,.45};
    double thetasI[s] = {-1.15*3*M_PI/4,-10,-10,-10,-10}; //s
    double forwardI[s] = {1,1,1,1,1};
    for (int i=0; i<s;i++) {
    	saShed->x_goals[i] = x_goalsI[i];
    	saShed->y_goals[i] = y_goalsI[i];
        saShed->thetas[i] = thetasI[i];
        saShed->forward[i] = forwardI[i];
    }
    printf("sashed initialized\n");
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

    set_param_normal(cvs);

    double x = mp->x; double y = mp->y;

    switch(saShed->status){
        case S0_sas:
        	if(saShed->go){
        		saShed->status = Dpmt1_sas;
                set_goal(cvs,saShed->x_goals[0],saShed->y_goals[0],saShed->thetas[0]);
        		printf("go to dp1\n");
        		saShed->go = 0;
        	}
            break;

        case Dpmt1_sas:{
    		sendFromHLCPF(cvs,cvs->saShed->forward[0]);
        	if(hlcPF->output){
        		saShed->status = Dpmt2_sas;
                set_goal(cvs,saShed->x_goals[1],saShed->y_goals[1],saShed->thetas[1]);
        	}
        	break;
        }

        case Dpmt2_sas:{
            //double sig = cvs->mlcPF->sigma;
            //cvs->mlcPF->sigma = .2;
            //set_param_prec(cvs);
            sendFromHLCPF(cvs,cvs->saShed->forward[1]);
            //cvs->mlcPF->sigma = sig;
            if(hlcPF->output){
                saShed->status = servoShedOut_sas;
                printf("go to servoShedout_ps\n");
            }
            break;
        }

        case servoShedOut_sas: {
            teensy_send(cvs, "A");
            set_goal(cvs,saShed->x_goals[2],saShed->y_goals[2],saShed->thetas[2]);
            printf("go to Dpmt3_ps\n");
            saShed->status = Dpmt3_sas;
            saShed->output =1;
            break;
        }


        case Dpmt3_sas:{
            sendFromHLCPF(cvs,cvs->saShed->forward[2]);
            teensy_send(cvs, "A");
            if(hlcPF->output){
                saShed->status = Dpmt4_sas;
                set_goal(cvs,saShed->x_goals[3],saShed->y_goals[3]);
                printf("go to Dpmt4_ps\n");
            }
            break;
        }

        case Dpmt4_sas:{
            double sig = cvs->mlcPF->sigma;
            cvs->mlcPF->sigma = 200;
            teensy_send(cvs, "A");
            sendFromHLCPF(cvs,cvs->saShed->forward[3]);
            cvs->mlcPF->sigma = sig;
            if(hlcPF->output){
                saShed->status = Dpmt5_sas;
                set_goal(cvs,saShed->x_goals[4],saShed->y_goals[4]);
                printf("go to Dpmt5_ps\n");
            }
            break;
        }

        case Dpmt5_sas:{
            double sig = cvs->mlcPF->sigma;
            cvs->mlcPF->sigma = 200;
            teensy_send(cvs, "A");
            sendFromHLCPF(cvs,cvs->saShed->forward[4]);
            cvs->mlcPF->sigma = sig;
            if(hlcPF->output){
                saShed->status = servoShedIn_sas;
                //set_goal(cvs,saShed->x_goals[3],saShed->y_goals[3]);
                printf("go to Dpmt2_ps\n");
            }
            break;
        }

        case servoShedIn_sas: {
            teensy_send(cvs, "B");
            inputs->t = inputs->t + 1;
            usleep(1000000);
            printf("go to end_ps\n");
            saShed->status = S0_sas;
            saShed->output = 1;
            break;
        }

        default:
            printf("Problem default value in FSM\n");
    }

}
