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
        Check4_es,Dpmt5_prec_es,Check5_es,Dpmt6_prec_es,Check6_es,Dpmt7_prec_es,Check7_es,Out_es,
    Dpmt5_noR_es,Check5_noR_es,Dpmt6_noR_es,Check6_noR_es,Dpmt7_noR_es,
    rec1_es,rec2_es,rec3_es,rec4_es,rec5_es,rec6_es,rec7_es};

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
                else set_goal(cvs,.65,0.43,2.1);
                printf("go to dp1\n");
                excSq->go = 0;
            }
            break;
        }

        case Dpmt1_es:{
            sendFromHLCPF(cvs,-1);
            if(hlcPF->output){
                set_commands(cvs,0,0);
                excSq->status = Dpmt2_es;
                if (TEAM) set_goal(cvs,2.5,0.2,M_PI);
                else set_goal(cvs,.76,.2,M_PI);
            }
            break;
        }

        case Dpmt2_es:{
            sendFromHLCPF(cvs,-1,1);
            if(hlcPF->output){
                set_commands(cvs,0,0);
                excSq->status = rec1_es;
                if (TEAM) set_goal(cvs,2.45,0.21,M_PI);
                else set_goal(cvs,.76,.2,M_PI);
            }
            break;
        }
        case rec1_es:{
            if (rec_static(cvs)) {
                printf("rec1 END : go to Dpmt1_prec_es\n");
                excSq->status = Dpmt1_prec_es;
            }
            break;
        }
        case Dpmt1_prec_es:{
            //set_param_normal(cvs);
            set_param_prec(cvs);
            hlcPF->Tau_max = .15;
            hlcPF->Tau_min = .1;
            mlcPF->sigma = 0.5;
            sendFromHLCPF(cvs,-1,1);
            if(hlcPF->output){
                set_commands(cvs,0,0);
                excSq->status = Check1_es;
                setChrono(cvs,2);
                teensy_send(cvs, "C");
                printf("go to check1\n");
                //excSq->output = 1;
            }
            break;
        }
        case Check1_es:{
            if (teensy->R1) {
                teensy->R_mes1 = 1;
            }
            else if (teensy->R3) {
                teensy->R_mes1 = 3;
            }
            if(checkChrono(cvs)){
                teensy->no_R = 0;
                teensy->R1 = 0;
                teensy->R2 = 0;
                teensy->R3 = 0;
                excSq->status = rec2_es;
                printf("go to Dpmt2_prec_es\n");
                if (TEAM) set_goal(cvs,2.265,0.21,M_PI);
                else set_goal(cvs,.97,.2,M_PI);
            }
            break;
        }
        case rec2_es:{
            if (rec_static(cvs)) {
                printf("rec1 END : go to Dpmt2_prec_es\n");
                excSq->status = Dpmt2_prec_es;
            }
            break;
        }
        case Dpmt2_prec_es:{
            //set_param_normal(cvs);
            set_param_prec(cvs);
            hlcPF->Tau_max = .15;
            hlcPF->Tau_min = .1;
            mlcPF->sigma = 0.5;
            sendFromHLCPF(cvs,-1,1);
            if(hlcPF->output){
                set_commands(cvs,0,0);
                excSq->status = Check2_es;
                teensy_send(cvs, "T");
                setChrono(cvs,2);
                printf("go to rec2_es\n");
            }
            break;
        }
        case Check2_es:{
            if(checkChrono(cvs)){
                printf("1 OR 3 :\n");
                if (teensy->R_mes1 == 1) {
                    excSq->status = rec4_es;
                    printf("1 : go to Dpmt4_prec_es\n");
                    if (TEAM) set_goal(cvs,1.895-0.01,0.21,M_PI);
                    else set_goal(cvs,3-1.895,M_PI);
                }
                else if (teensy->R_mes1 == 3) {
                    excSq->status = rec3_es;
                    printf("3 : go to Dpmt3_prec_es\n");
                    if (TEAM) set_goal(cvs,2.08-0.01,0.21,M_PI);
                    else set_goal(cvs,3-2.08,.2,M_PI);
                }
            }
            break;
        }
        case rec3_es:{
            if (rec_static(cvs)) {
                printf("rec2 END : go to Dpmt3_prec_es\n");
                excSq->status = Dpmt3_prec_es;
            }
            break;
        }
        case Dpmt3_prec_es:{
            //set_param_normal(cvs);
            set_param_prec(cvs);
            hlcPF->Tau_max = .15;
            hlcPF->Tau_min = .1;
            mlcPF->sigma = 0.5;
            sendFromHLCPF(cvs,-1,1);
            if(hlcPF->output){
                set_commands(cvs,0,0);
                excSq->status = Check3_es;
                setChrono(cvs,2);
                teensy_send(cvs, "T");
                printf("go to check3\n");
            }
            break;
        }
        case Check3_es:{
            if(checkChrono(cvs)){
                excSq->status = rec4_es;
                printf("go to Dpmt4_es\n");
                if (TEAM) set_goal(cvs,1.895-0.01,0.21,M_PI);
                else set_goal(cvs,3-1.895,.2,M_PI);
            }
            break;
        }
        case rec4_es:{
            if (rec_static(cvs)) {
                printf("rec2 END : go to Dpmt4_prec_es\n");
                excSq->status = Dpmt4_prec_es;
            }
            break;
        }
        case Dpmt4_prec_es:{
            //set_param_normal(cvs);
            set_param_prec(cvs);
            hlcPF->Tau_max = .15;
            hlcPF->Tau_min = .1;
            mlcPF->sigma = 0.5;
            sendFromHLCPF(cvs,-1,1);
            if(hlcPF->output){
                set_commands(cvs,0,0);
                excSq->status = Check4_es;
                setChrono(cvs,2);
                teensy_send(cvs, "C");
                printf("go to check4\n");
            }
            break;
        }
        case Check4_es:{
            if (teensy->R1) {
                teensy->R_mes4 = 1;
            }
            else if (teensy->R2) {
                teensy->R_mes4 = 2;
            }
            else if (teensy->no_R) {
                teensy->R_mes4 = 0;
            }
            if(checkChrono(cvs)){
                teensy->no_R = 0;
                teensy->R1 = 0;
                teensy->R2 = 0;
                teensy->R3 = 0;
                printf("1 OR 2 :\n");
                if (teensy->R_mes4 == 1) {
                    excSq->status = rec7_es;
                    printf("1 : go to Dpmt7_es\n");
                    if (TEAM) set_goal(cvs,1.34-0.01,0.21,M_PI);
                    else set_goal(cvs,1.34,.2,M_PI);
                }
                else if (teensy->R_mes4 == 2) {
                    excSq->status = rec5_es;
                    printf("2 : go to Dpmt5_es\n");
                    if (TEAM) set_goal(cvs,1.71-0.01,0.21,M_PI);
                    else set_goal(cvs,1.71,.2,M_PI);
                }
                else if (teensy->R_mes4 == 0) {
                    excSq->status = Dpmt5_noR_es;
                    printf("2 : go to Dpmt5_noR_es\n");
                    if (TEAM) set_goal(cvs,1.71-0.01,0.21,M_PI);
                    else set_goal(cvs,1.71,.2,M_PI);
                }
            }
            break;
        }
        case rec5_es:{
            if (rec_static(cvs)) {
                printf("rec2 END : go to Dpmt5_prec_es\n");
                excSq->status = Dpmt5_prec_es;
            }
            break;
        }
        case Dpmt5_prec_es:{
            //set_param_normal(cvs);
            set_param_prec(cvs);
            hlcPF->Tau_max = .15;
            hlcPF->Tau_min = .1;
            mlcPF->sigma = 0.5;
            sendFromHLCPF(cvs,-1,1);
            if(hlcPF->output){
                set_commands(cvs,0,0);
                excSq->status = Check5_es;
                setChrono(cvs,2);
                teensy_send(cvs, "T");
                printf("go to check5\n");
            }
            break;
        }
        case Dpmt5_noR_es:{
            //set_param_normal(cvs);
            set_param_prec(cvs);
            hlcPF->Tau_max = .15;
            hlcPF->Tau_min = .1;
            mlcPF->sigma = 0.5;
            sendFromHLCPF(cvs,-1,1);
            if(hlcPF->output){
                set_commands(cvs,0,0);
                excSq->status = Check5_noR_es;
                setChrono(cvs,2);
                teensy_send(cvs, "C");
                printf("go to check1\n");
            }
            break;
        }
        case Check5_noR_es:{
            if (teensy->R1) {
                teensy->R_mes5 = 1;
            }
            else if (teensy->R2) {
                teensy->R_mes5 = 2;
            }
            else if (teensy->no_R) {
                teensy->R_mes5 = 0;
            }
            if(checkChrono(cvs)){
                teensy->no_R = 0;
                teensy->R1 = 0;
                teensy->R2 = 0;
                teensy->R3 = 0;
                printf("1 OR 2 :\n");
                if (teensy->R_mes5 == 1) {
                    excSq->status = rec6_es;
                    printf("1 : go to Dpmt6_es\n");
                    if (TEAM) set_goal(cvs,1.525-0.01,0.21,M_PI);
                    else set_goal(cvs,3-1.525,.2,M_PI);
                }
                else if (teensy->R_mes5 == 2) {
                    excSq->status = rec7_es;
                    printf("2 : go to Dpmt7_es\n");
                    if (TEAM) set_goal(cvs,1.34-0.01,0.21,M_PI);
                    else set_goal(cvs,1.34,.2,M_PI);
                }
                else if (teensy->R_mes5 == 0) {
                    excSq->status = Dpmt6_noR_es;
                    printf("2 : go to Dpmt6_noR_es\n");
                    if (TEAM) set_goal(cvs,1.525-0.01,0.21,M_PI);
                    else set_goal(cvs,3-1.525,.2,M_PI);
                }
                teensy->R1 = 0;
                teensy->R2 = 0;
                teensy->R3 = 0;
            }
            break;
        }
        case Check5_es:{
            if(checkChrono(cvs)){
                excSq->status = rec6_es;
                printf("go to Dpmt6_es\n");
                if (TEAM) set_goal(cvs,1.525-0.01,0.21,M_PI);
                else set_goal(cvs,3-1.525,.2,M_PI);
            }
            break;
        }
        case rec6_es:{
            if (rec_static(cvs)) {
                printf("rec2 END : go to Dpmt6_prec_es\n");
                excSq->status = Dpmt6_prec_es;
            }
            break;
        }
        case Dpmt6_prec_es:{
            //set_param_normal(cvs);
            set_param_prec(cvs);
            hlcPF->Tau_max = .15;
            hlcPF->Tau_min = .1;
            mlcPF->sigma = 0.5;
            sendFromHLCPF(cvs,-1,1);
            if(hlcPF->output){
                set_commands(cvs,0,0);
                excSq->status = Check6_es;
                setChrono(cvs,2);
                teensy_send(cvs, "T");
                printf("go to check6\n");
            }
            break;
        }
        case Dpmt6_noR_es:{
            //set_param_normal(cvs);
            set_param_prec(cvs);
            hlcPF->Tau_max = .15;
            hlcPF->Tau_min = .1;
            mlcPF->sigma = 0.5;
            sendFromHLCPF(cvs,-1,1);
            if(hlcPF->output){
                set_commands(cvs,0,0);
                excSq->status = Check6_noR_es;
                setChrono(cvs,2);
                teensy_send(cvs, "C");
                printf("go to check6_noR\n");
            }
            break;
        }
        case Check6_noR_es:{
            if (teensy->R1) {
                teensy->R_mes6 = 1;
            }
            else if (teensy->R2) {
                teensy->R_mes6 = 2;
            }
            else if (teensy->no_R) {
                teensy->R_mes6 = 0;
            }
            if(checkChrono(cvs)){
                teensy->no_R = 0;
                teensy->R1 = 0;
                teensy->R2 = 0;
                teensy->R3 = 0;
                printf("1 OR 2 :\n");
                if (teensy->R_mes6 == 1) {
                    excSq->status = Out_es;
                    printf("go to Out_es\n");
                    if (TEAM) set_goal(cvs,0.9,0.4,M_PI/2);
                    else set_goal(cvs,3-.8,.4,M_PI/2);
                }
                else if (teensy->R_mes6 == 2) {
                    excSq->status = rec7_es;
                    printf("2 : go to Dpmt7_es\n");
                    if (TEAM) set_goal(cvs,1.34-0.01,0.21,M_PI);
                    else set_goal(cvs,1.34,.2,M_PI);
                }
                else if (teensy->R_mes6 == 0) {
                    excSq->status = Dpmt7_noR_es;
                    printf("2 : go to Dpmt7_noR_es\n");
                    if (TEAM) set_goal(cvs,1.34-0.01,0.21,M_PI);
                    else set_goal(cvs,1.34,.2,M_PI);
                }
                teensy->R1 = 0;
                teensy->R2 = 0;
                teensy->R3 = 0;
            }
            break;
        }
        case Check6_es:{
            if(checkChrono(cvs)){
                excSq->status = Out_es;
                printf("go to Out_es\n");
                if (TEAM) set_goal(cvs,0.9,0.4,M_PI/2);
                else set_goal(cvs,3-.8,.4,M_PI/2);
            }
            break;
        }
        case rec7_es:{
            if (rec_static(cvs)) {
                printf("rec2 END : go to Dpmt7_prec_es\n");
                excSq->status = Dpmt7_prec_es;
            }
            break;
        }
        case Dpmt7_prec_es:{
            //set_param_normal(cvs);
            set_param_prec(cvs);
            hlcPF->Tau_max = .15;
            hlcPF->Tau_min = .1;
            mlcPF->sigma = 0.5;
            sendFromHLCPF(cvs,-1,1);
            if(hlcPF->output){
                set_commands(cvs,0,0);
                excSq->status = Check7_es;
                setChrono(cvs,2);
                teensy_send(cvs, "T");
                printf("go to check1\n");
            }
            break;
        }
        case Dpmt7_noR_es:{
            //set_param_normal(cvs);
            set_param_prec(cvs);
            hlcPF->Tau_max = .15;
            hlcPF->Tau_min = .1;
            mlcPF->sigma = 0.5;
            sendFromHLCPF(cvs,-1,1);
            if(hlcPF->output){
                set_commands(cvs,0,0);
                excSq->status = Check7_es;
                setChrono(cvs,2);
                teensy_send(cvs, "C");
                printf("go to check1\n");
            }
            break;
        }
        case Check7_es:{
            if(checkChrono(cvs)){
                excSq->status = Out_es;
                printf("go to Out_es\n");
                if (TEAM) set_goal(cvs,0.9,0.4,M_PI/2);
                else set_goal(cvs,3-.8,.4,M_PI/2);
            }
            break;
        }
        case Out_es:{
            set_param_normal(cvs);
            sendFromHLCPF(cvs,-1);
            if(hlcPF->output){
                set_commands(cvs,0,0);
                printf("END\n");
                excSq->output = 1;
            }
            break;
        }
        default:
            printf("probleme defautl value in FSM\n");
            excSq->output = 1;
    }

}