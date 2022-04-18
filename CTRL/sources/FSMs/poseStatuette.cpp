#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <cmath> //for sqrt

#include "statuetteAndShed.h"
#include "FSMs_utils.h"


//angle : opposé + pi

enum {S0_pos,Go_to_vitrine_pos,vitrine_prec1_pos,vitrine_prec2_pos,dpmtprec_pos,drop_statuette_pos};

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
        	if(poseStat->go){
        		poseStat->status = Go_to_vitrine_pos;
        		printf("go to dp1\n");
        		poseStat->go = 0;
                if (TEAM) set_goal(cvs,2.65,1.35,0.8*M_PI/2);
                else set_goal(cvs,3-2.5,.6,-10);    
                poseStat->status = Go_to_vitrine_pos;
                break;
        	}
            break;
        
        case Go_to_vitrine_pos:{
            set_param_normal(cvs);
            sendFromHLCPF(cvs,-1);
            if(hlcPF->output){
                poseStat->status = vitrine_prec1_pos;
                if (TEAM) set_goal(cvs,2.8,1.7,M_PI/2);
                else set_goal(cvs,3-2.5,.6,-10);    
                printf("go to vitrine_prec_pos\n");
            }
            break;
        }

        case vitrine_prec1_pos:{
            set_param_prec(cvs);
            sendFromHLCPF(cvs,1,1);
            if(hlcPF->output){
                printf("ended\n");
                poseStat->status = vitrine_prec2_pos;
                if (TEAM) set_goal(cvs,2.8,1.9,-10);
                else set_goal(cvs,3-2.5,.6,-10); 
            }
            break;
        }

        case vitrine_prec2_pos:{
            set_param_prec(cvs);
            sendFromHLCPF(cvs,1,1);
            teensy_recv(cvs);
            if (teensy->switch_F) {
                motors_stop(cvs);
                teensy->switch_F = 0;
                printf("go to Dpmt7_ps\n");
                poseStat->output =1;
            }
            break;
        }

        case drop_statuette_pos:{
            break;
        }


        default:
            printf("Problem default value in FSM\n");
    }

}
