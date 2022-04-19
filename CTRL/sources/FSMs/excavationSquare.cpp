#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <cmath> //for sqrt

#include "excavationSquare.h"
#include "FSMs_utils.h"


//4 possibilités 
//carrés peuvent avoir ete retounré par l'ennemi
//attention a l'ennemi


enum {S0_es,Dpmt1_es,Dpmt2_es,Dpmt3_es,Check1_es,Dpmt4_es};

void excSq_init(excSquares *excSq){
    excSq->status = S0_ps;
    excSq->output = 0;
    excSq->go = 0;
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

    set_param_normal(cvs);
    int TEAM = cvs->inputs->team;

    //only if usefull
    double pos[5];
    double x, y, th;
    get_pos(cvs, pos);
    th = pos[2];
    x = pos[0];//+ hlcPF->x_shift * cos(th);
    y = pos[1];//+ hlcPF->x_shift * sin(th);


    switch(excSq->status){
        case S0_es:{
            if(excSq->go){
                excSq->status = Dpmt1_es;
                if (TEAM) set_goal(cvs,2.5,0.5,M_PI/2);
                else set_goal(cvs,3-2.44,1.55,limit_angle(2.71+M_PI));
                printf("go to dp1\n");
                excSq->go = 0;
            }
            break;
        }

        case Dpmt1_es:{
            sendFromHLCPF(cvs,-1);
            if(hlcPF->output){
                excSq->status = Dpmt2_es;
                if (TEAM) set_goal(cvs,2.47,0.23,M_PI);
                else set_goal(cvs,3-2.3,1.5,-10);
            }
            break;
        }

        case Dpmt2_es:{
            sendFromHLCPF(cvs,-1,1);
            if(hlcPF->output){
                excSq->status = Dpmt3_es;
                if (TEAM) set_goal(cvs,2.46,0.21,M_PI);
                else set_goal(cvs,3-2.3,1.5,-10);
            }
            break;
        }

        case Dpmt3_es:{
            set_param_prec(cvs);
            sendFromHLCPF(cvs,-1,1);
            if(hlcPF->output){
                excSq->status = Check1_es;
                setChrono(cvs,2);
                teensy_send(cvs, "C");
                printf("go to chec1\n");
            }
            break;
        }



        case Check1_es:{
            teensy_recv(cvs);
            if(checkChrono(cvs)){
                excSq->status = Dpmt4_es;
                printf("go to servoShedIn_sas\n");
                if (TEAM) set_goal(cvs,2.275,0.21,M_PI);
                else set_goal(cvs,3-2.3,1.5,-10);
            }
            break;
        }

        case Dpmt4_es:{
            set_param_prec(cvs);
            sendFromHLCPF(cvs,-1,1);
            if(hlcPF->output){
                excSq->status = Check1_es;
                setChrono(cvs,2);
                teensy_send(cvs, "C");
                printf("go to chec1\n");
            }
            break;
        }


        default:
            printf("probleme defautl value in FSM\n");
    }

}