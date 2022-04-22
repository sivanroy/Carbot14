#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <cmath> //for sqrt

#include "statuetteAndShed.h"
#include "FSMs_utils.h"


//angle : opposé + pi

enum {S0_pos,Go_to_vitrine_pos,vitrine_prec1_pos,vitrine_prec2_pos,dpmtprec_pos,drop_statuette_pos,go_back_prec_pos,
        rec_pos,rec_out_pos};

void poseStat_init(poseStatuette *poseStat) {
    poseStat->status = S0_ps;
    poseStat->output = 0;
    poseStat->go = 0;
    printf("poseStat initialized\n");
}

void poseStat_launch(ctrlStruct *cvs){
	cvs->poseStat->go = 1;
	cvs->poseStat->status = S0_ps;
}


void poseStat_loop(ctrlStruct *cvs){
	poseStatuette *poseStat = cvs->poseStat;
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



    switch(poseStat->status){
        case S0_pos:
        	//if(poseStat->go){
        		poseStat->status = Go_to_vitrine_pos;
        		printf("go to dp1\n");
        		poseStat->go = 0;
                if (TEAM) set_goal(cvs,2.58,1.25,1.4*M_PI/4);//2.65,1.35,0.6*M_PI/2
                else set_goal(cvs,.4,1.25,limit_angle(-1.4*M_PI/4 + M_PI));
                poseStat->status = Go_to_vitrine_pos;
                break;
        	//
            break;
        
        case Go_to_vitrine_pos:{
            set_param_normal(cvs);
            sendFromHLCPF(cvs,-1);
            if(hlcPF->output){
                motors_stop(cvs);
                set_commands(cvs,0,0);
                poseStat->status = rec_pos;

            }
            break;
        }
        case rec_pos:{
            if (rec_static(cvs)) {
                poseStat->status = vitrine_prec1_pos;
                if (TEAM) set_goal(cvs,2.78,1.7,M_PI/2);
                else set_goal(cvs,.25,1.7,M_PI/2);
                printf("go to vitrine_prec_pos\n");
            }
            break;
        }

        case vitrine_prec1_pos:{
            set_param_prec(cvs);
            sendFromHLCPF(cvs,1,1);
            if(hlcPF->output){
                motors_stop(cvs);
                set_commands(cvs,0,0);
                printf("ended\n");
                poseStat->status = vitrine_prec2_pos;
                if (TEAM) set_goal(cvs,2.78,1.99,-10);
                else set_goal(cvs,.25,1.99,-10);
            }
            break;
        }

        case vitrine_prec2_pos:{
            set_param_prec(cvs);
            hlcPF->Tau_max = .15;
            hlcPF->Tau_min = .1;
            mlcPF->sigma = 0.5;
            sendFromHLCPF(cvs,1,1);
            teensy_recv(cvs);
            if (teensy->switch_F) {
                motors_stop(cvs);
                teensy_send(cvs, "Q");
                arduino_send(cvs,"K");
                setChrono(cvs,1.2);
                teensy->switch_F = 0;
                poseStat->status = drop_statuette_pos;
                printf("go to Dpmt7_ps\n");
            }
            break;
        }

        case drop_statuette_pos:{
            if (checkChrono(cvs)) {
                poseStat->status = go_back_prec_pos;
                if (TEAM) set_goal(cvs,2.8,1.6,-1.2*M_PI/2);
                else set_goal(cvs,.25,1.6,0);
                printf("go to Dpmt3_ps\n");
            }
            break;
        }
        case go_back_prec_pos:{
            set_param_prec(cvs);
            sendFromHLCPF(cvs,0,1);
            if(hlcPF->output){
                poseStat->status = rec_out_pos;
                printf("rec START\n");
            }
            break;
        }
        case rec_out_pos:{
            if (rec_static(cvs)) {
                printf("rec END\n");
                poseStat->output =1;
            }
            break;
        }
        default:
            printf("Problem default value in FSM\n");
    }

}
