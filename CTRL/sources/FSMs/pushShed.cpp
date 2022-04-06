#include "pushShed.h"


#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <cmath> //for sqrt

#include "pushShed.h"
#include "FSMs_utils.h"



//enum enum {S0_ps,Dpmt1_ps,Dpmt2_ps,Close_ps,Dpmt3_ps,Dpmt4_ps,Dpmt5_ps,Push_ps,Ok_ps,NotOk_ps};

void pushShed_init(pushShed *pshed) {
    pshed->status = S0_ps;
    pshed->output = 0;
    pshed->go = 1;

    int s = 7; //2.28 ;; 1.51
    double x_goalsI[s] = {2.28,2.2,2.25,2.52,2.7,2.72,1};
    double y_goalsI[s] = {1.51,1.12,0.85,0.52,0.35,1.3,0.75};
    double thetasI[s] = {-10,-10,-10,-10,-10,-10,-10}; //s
    double forwardI[s] = {1,1,1,1,1,1,-1};
    for (int i=0; i<s;i++) {
    	pshed->x_goals[i] = x_goalsI[i];
    	pshed->y_goals[i] = y_goalsI[i];
        pshed->thetas[i] = thetasI[i];
        pshed->forward[i] = forwardI[i];
    }
}

void pushShed_launch(ctrlStruct *cvs){
	cvs->pshed->go = 1;
	cvs->pshed->status = S0_ps;
}


void pushShed_loop(ctrlStruct *cvs){
	pushShed *pshed = cvs->pshed;
    myPosition *mp = cvs->mp;
    ctrlIn  *inputs = cvs->inputs;
    highLevelCtrlPF *hlcPF = cvs->hlcPF;
    midLevelCtrlPF *mlcPF = cvs->mlcPF;
    midLevelCtrl *mlc = cvs->mlc;
    teensyStruct *teensy = cvs->teensy;


    double x = mp->x; double y = mp->y;
    double xg; double yg;

    switch(pshed->status){
        case S0_ps:
        	if(pshed->go){
        		pshed->status = Dpmt1_ps;
        		printf("go to dp1\n");
        		pshed->go = 0;
                set_goal(cvs,pshed->x_goals[0],pshed->y_goals[0]);
        	}
            break;

        case Dpmt1_ps:{
    		sendFromHLCPF(cvs,cvs->pshed->forward[0]);
        	if(hlcPF->output){
        		pshed->status = Dpmt2_ps;
                set_goal(cvs,pshed->x_goals[1],pshed->y_goals[1]);
        		printf("go to Dpmt2_ps\n");
        	}
        	break;
        }

        case Dpmt2_ps:{
    		sendFromHLCPF(cvs);
        	if(hlcPF->output){
                pshed->output=1;
        		//pshed->status = servoShedOut;
        		printf("go to dp3\n");
        	}
        	break;
        }
        case servoShedOut: {
            teensy_send(cvs, "A");
            inputs->t = inputs->t + 2;
            usleep(2000000);
            pshed->status = Dpmt3_ps;
            break;
        }

        case Dpmt3_ps:{
            mlcPF->sigma = 1;
            mlcPF->max_sp_ref = 2.5;
            mlcPF->min_sp_ref = -2.5;
            xg = pshed->x_goals[2];
        	yg = pshed->y_goals[2];
        	sendFromHLCPF(cvs);//,xg,yg);
        	if(hlcPF->output){
        		pshed->status = Dpmt4_ps;
        		printf("go to dp4\n");
        	}
        	break;
        }

        case Dpmt4_ps:{
            xg = pshed->x_goals[3];
        	yg = pshed->y_goals[3];
        	sendFromHLCPF(cvs);//,xg,yg);
        	if(hlcPF->output){
        		pshed->status = Dpmt5_ps;
        		motors_stop(cvs);
        		printf("go to Push_ps\n");
        	}
        	break; 
        }
        case Dpmt5_ps:{
            xg = pshed->x_goals[4];
            yg = pshed->y_goals[4];
            sendFromHLCPF(cvs);//,xg,yg);
            if(hlcPF->output){
                pshed->status = servoShedIn;
                motors_stop(cvs);
                printf("go to Push_ps\n");
            }
            break;
        }
        case servoShedIn:{
            teensy_send(cvs, "B");
            //inputs->t = inputs->t + 2;
            //usleep(2000000);
            pshed->status = Dpmt6_ps;
            break;
        }
        case Dpmt6_ps:{
            xg = pshed->x_goals[5];
            yg = pshed->y_goals[5];
            sendFromMLC(cvs, xg, yg);
            teensy_recv(cvs);
            if((teensy->switch_F && teensy->switch_F_end == 0)){
                printf("switch_F : %d\n", teensy->switch_F);
                motors_stop(cvs);
                teensy_send(cvs, "5");
                teensy->switch_F = 0;
                teensy->switch_F_end = 1;

                inputs->t = inputs->t + 5;
                usleep(5000000);

                mp->th = -M_PI/4;
                mp->x = 2.605;
                mp->y = 0.395;
                pshed->status = Back;
                mlcPF->t_start = inputs->t;
                printf("go to Back\n");
            }
            break;
        }
        case Back:{
            sendFromMLCPF(cvs, -0.1, -M_PI/4);
            if (inputs->t >= mlcPF->t_start + 2) {
                motors_stop(cvs);
                pshed->status = GoHome;
                mlcPF->t_start = 0;
            }
            break;
        }
        case GoHome:{
            mlcPF->sigma = 0.45;
            mlcPF->max_sp_ref = 5;
            mlcPF->min_sp_ref = -5;
            xg = pshed->x_goals[6];
            yg = pshed->y_goals[6];
            sendFromHLCPF(cvs);//,xg,yg);
            if(hlcPF->output){
                pshed->status = Close_ps;
                motors_stop(cvs);
                printf("go to Push_ps\n");
            }
            break;
        }

        case Close_ps:{
            printf("End simu\n");
            //pshed->status = Dpmt3_ps;
            break;
        }

        case Push_ps:
        	//pshed->status = S0_ps;
        	pshed->output = 1;
        	printf("No feedback given yet into pushSHed\n");
        	break;

        default:
            printf("probleme defautl value in FSM\n");
    }

}







/////ESQUIVE LOOP

void esquive_loop(ctrlStruct *cvs) {
    pushShed *pshed = cvs->pshed;
    myPosition *mp = cvs->mp;
    ctrlIn  *inputs = cvs->inputs;
    highLevelCtrlPF *hlcPF = cvs->hlcPF;
    midLevelCtrlPF *mlcPF = cvs->mlcPF;
    midLevelCtrl *mlc = cvs->mlc;
    teensyStruct *teensy = cvs->teensy;

    double x = mp->x; double y = mp->y;
    double xg; double yg;

    switch(pshed->status){
        case S0_ps:{
            if(pshed->go){
                pshed->status = Dpmt1_es;
                printf("go to dp1\n");
                pshed->go = 0;
                set_goal(cvs,pshed->x_goals[6],pshed->y_goals[6]);
            }
            break;
        }
        case Dpmt1_es:{
            //printf("D1\n");
            sendFromHLCPF(cvs);
            if(hlcPF->output){
                pshed->status = Close_es;
                printf("Close\n");
            }

            break;
        }
        case Close_es:{
            pshed->output = 1;
            //printf("End simu\n");
            break;
        }
    }
}