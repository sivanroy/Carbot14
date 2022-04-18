#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <cmath> //for sqrt

#include "excavationSquare.h"
#include "FSMs_utils.h"


//4 possibilités 
//carrés peuvent avoir ete retounré par l'ennemi
//attention a l'ennemi


enum {S0_es,Dpmt1_es,Dpmt2_es,Dpmt3_es,servoShedOut_es};

void excSq_init(excSquares *excSq){
    excSq->status = S0_ps;
    excSq->output = 0;
    excSq->go = 0;

    int s = 3; //2.28 ;; 1.51
    double x_goalsI[s] = {2.4,1,1};
    double y_goalsI[s] = {0.5,1,1};
    double thetasI[s] = {-M_PI/2,-10,-10}; //s
    double forwardI[s] = {-1,-1,-1};
    for (int i=0; i<s;i++) {
    	excSq->x_goals[i] = x_goalsI[i];
    	excSq->y_goals[i] = y_goalsI[i];
        excSq->thetas[i] = thetasI[i];
        excSq->forward[i] = forwardI[i];
    }
    printf("excSq initialized\n");
}

void excSq_launch(ctrlStruct *cvs){
	cvs->excSq->go = 1;
	cvs->excSq->status = S0_ps;
    cvs->excSq->output = 0;
}


void excSq_loop(ctrlStruct *cvs){
	excSquares *excSq = cvs->excSq;
    myPosition *mp = cvs->mp;
    ctrlIn  *inputs = cvs->inputs;
    highLevelCtrlPF *hlcPF = cvs->hlcPF;
    midLevelCtrlPF *mlcPF = cvs->mlcPF;
    midLevelCtrl *mlc = cvs->mlc;
    teensyStruct *teensy = cvs->teensy;

    //only if usefull
    double pos[5];
    double x, y, th;
    get_pos(cvs, pos);
    th = pos[2];
    x = pos[0];//+ hlcPF->x_shift * cos(th);
    y = pos[1];//+ hlcPF->x_shift * sin(th);


    switch(excSq->status){
        case S0_es:
        	if(excSq->go){
        		excSq->status = Dpmt1_es;
                set_goal(cvs,excSq->x_goals[0],excSq->y_goals[0]);
        		printf("go to dp1\n");
        		excSq->go = 0;
        	}
            break;

        case Dpmt1_es:{
    		sendFromHLCPF(cvs,cvs->excSq->forward[0]);
        	if(hlcPF->output){
        		excSq->status = servoShedOut_es;
        	}
        	break;
        }

        case servoShedOut_es: {
            teensy_send(cvs, "A");
            inputs->t = inputs->t + 2;
            usleep(2000000);
            excSq->status = Dpmt2_es;
            set_goal(cvs,excSq->x_goals[1],excSq->y_goals[1]);
            printf("go to Dpmt2_ps\n");
            break;
        }

        case Dpmt2_es:{
            sendFromHLCPF(cvs,cvs->excSq->forward[1]);
            if(hlcPF->output){
                excSq->status = Dpmt3_es;
                set_goal(cvs,excSq->x_goals[2],excSq->y_goals[2]);
                printf("go to Dpmt2_ps\n");
            }
            break;
        }

        case Dpmt3_es:{
            sendFromHLCPF(cvs,cvs->excSq->forward[1]);
            if(hlcPF->output){
                excSq->status = S0_es;
                excSq->output = 1;
                printf("go to Dpmt2_ps\n");
            }
            break;
        }
        default:
            printf("probleme defautl value in FSM\n");
    }

}