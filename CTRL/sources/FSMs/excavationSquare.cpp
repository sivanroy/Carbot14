#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <cmath> //for sqrt

#include "excavationSquare.h"
#include "FSMs_utils.h"


//4 possibilités 
//carrés peuvent avoir ete retounré par l'ennemi
//attention a l'ennemi


enum {S0_es,Dpmt1_es,Dpmt2_es,Dpmt1_prec_es,Check1_es,Dpmt2_prec_es,Check2_es,Dpmt3_prec_es,Check3_es,Dpmt4_prec_es,
        Check4_es,Dpmt5_prec_es,Check5_es,Dpmt6_prec_es,Check6_es,Dpmt7_prec_es,Check7_es,Out_es};

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
                if (TEAM) set_goal(cvs,2.53,0.5,M_PI/2);
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
                if (TEAM) set_goal(cvs,2.5,0.2,M_PI);
                else set_goal(cvs,3-2.3,1.5,-10);
            }
            break;
        }

        case Dpmt2_es:{
            sendFromHLCPF(cvs,-1,1);
            if(hlcPF->output){
                excSq->status = Dpmt1_prec_es;
                if (TEAM) set_goal(cvs,2.45,0.2,M_PI);
                else set_goal(cvs,3-2.3,1.5,-10);
            }
            break;
        }

        case Dpmt1_prec_es:{
            set_param_prec(cvs);
            sendFromHLCPF(cvs,-1,1);
            if(hlcPF->output){
                excSq->status = Check1_es;
                setChrono(cvs,2);
                //teensy_send(cvs, "C");
                printf("go to check1\n");
            }
            break;
        }

        case Check1_es:{
            teensy_recv(cvs);
            if (teensy->R1) {
                teensy->R_mes1 = 1;
            }
            else if (teensy->R3) {
                teensy->R_mes1 = 3;
            }
            if(checkChrono(cvs)){
                teensy->R1 = 0;
                teensy->R2 = 0;
                teensy->R3 = 0;
                excSq->status = Dpmt2_prec_es;
                printf("go to Dpmt2_prec_es\n");
                if (TEAM) set_goal(cvs,2.265,0.2,M_PI);
                else set_goal(cvs,3-2.3,1.5,-10);
            }
            break;
        }
        case Dpmt2_prec_es:{
            set_param_prec(cvs);
            sendFromHLCPF(cvs,-1,1);
            if(hlcPF->output){
                excSq->status = Check2_es;
                setChrono(cvs,2);
                //teensy_send(cvs, "C");
                printf("go to check2\n");
            }
            break;
        }
        case Check2_es:{
            if(checkChrono(cvs)){
                if (teensy->R_mes1 == 1) {
                    excSq->status = Dpmt4_prec_es;
                    printf("go to Dpmt4_prec_es\n");
                    if (TEAM) set_goal(cvs,1.895,0.2,M_PI);
                    else set_goal(cvs,3-2.3,1.5,-10);
                }
                else if (teensy->R_mes1 == 3) {
                    excSq->status = Dpmt3_prec_es;
                    printf("go to Dpmt3_prec_es\n");
                    if (TEAM) set_goal(cvs,2.08,0.2,M_PI);
                    else set_goal(cvs,3-2.3,1.5,-10);
                }
            }
            break;
        }
        case Dpmt3_prec_es:{
            set_param_prec(cvs);
            sendFromHLCPF(cvs,-1,1);
            if(hlcPF->output){
                excSq->status = Check3_es;
                setChrono(cvs,2);
                //teensy_send(cvs, "C");
                printf("go to check3\n");
            }
            break;
        }
        case Check3_es:{
            if(checkChrono(cvs)){
                excSq->status = Dpmt4_prec_es;
                printf("go to Dpmt5_es\n");
                if (TEAM) set_goal(cvs,1.895,0.2,M_PI);
                else set_goal(cvs,3-2.3,1.5,-10);
            }
            break;
        }
        case Dpmt4_prec_es:{
            set_param_prec(cvs);
            sendFromHLCPF(cvs,-1,1);
            if(hlcPF->output){
                excSq->status = Check4_es;
                setChrono(cvs,2);
                //teensy_send(cvs, "C");
                printf("go to check4\n");
            }
            break;
        }
        case Check4_es:{
            teensy_recv(cvs);
            if (teensy->R1) {
                teensy->R_mes4 = 1;
            }
            else if (teensy->R2) {
                teensy->R_mes4 = 2;
            }
            if(checkChrono(cvs)){
                if (teensy->R_mes4 == 1) {
                    excSq->status = Dpmt7_prec_es;
                    printf("go to Dpmt7_es\n");
                    if (TEAM) set_goal(cvs,1.34,0.2,M_PI);
                    else set_goal(cvs,3-2.3,1.5,-10);
                }
                else if (teensy->R_mes4 == 2) {
                    excSq->status = Dpmt5_prec_es;
                    printf("go to Dpmt5_es\n");
                    if (TEAM) set_goal(cvs,1.71,0.2,M_PI);
                    else set_goal(cvs,3-2.3,1.5,-10);
                }
                teensy->R1 = 0;
                teensy->R2 = 0;
                teensy->R3 = 0;
            }
            break;
        }
        case Dpmt5_prec_es:{
            set_param_prec(cvs);
            sendFromHLCPF(cvs,-1,1);
            if(hlcPF->output){
                excSq->status = Check5_es;
                setChrono(cvs,2);
                //teensy_send(cvs, "C");
                printf("go to check1\n");
            }
            break;
        }
        case Check5_es:{
            if(checkChrono(cvs)){
                excSq->status = Dpmt6_prec_es;
                printf("go to Dpmt6_es\n");
                if (TEAM) set_goal(cvs,1.525,0.2,M_PI);
                else set_goal(cvs,3-2.3,1.5,-10);
            }
            break;
        }
        case Dpmt6_prec_es:{
            set_param_prec(cvs);
            sendFromHLCPF(cvs,-1,1);
            if(hlcPF->output){
                excSq->status = Check6_es;
                setChrono(cvs,2);
                //teensy_send(cvs, "C");
                printf("go to check1\n");
            }
            break;
        }
        case Check6_es:{
            if(checkChrono(cvs)){
                excSq->status = Out_es;
                printf("go to Out_es\n");
                if (TEAM) set_goal(cvs,0.8,0.4,M_PI/2);
                else set_goal(cvs,3-2.3,1.5,-10);
            }
            break;
        }
        case Dpmt7_prec_es:{
            set_param_prec(cvs);
            sendFromHLCPF(cvs,-1,1);
            if(hlcPF->output){
                excSq->status = Check7_es;
                setChrono(cvs,2);
                //teensy_send(cvs, "C");
                printf("go to check1\n");
            }
            break;
        }
        case Check7_es:{
            if(checkChrono(cvs)){
                excSq->status = Out_es;
                printf("go to Out_es\n");
                if (TEAM) set_goal(cvs,0.8,0.4,M_PI/2);
                else set_goal(cvs,3-2.3,1.5,-10);
            }
            break;
        }
        default:
            printf("probleme defautl value in FSM\n");
    }

}