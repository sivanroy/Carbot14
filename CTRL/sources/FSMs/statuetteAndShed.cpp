#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <cmath> //for sqrt

#include "statuetteAndShed.h"
#include "FSMs_utils.h"


//angle : opposé + pi

enum {S0_sas,Dpmt1_sas,servoShedOut_sas,Dpmt2_sas,Dpmt3_sas,Dpmt4_sas,Dpmt5_sas,Dpmt6_sas,Wait_for_5_sas,Dpmt7_sas,Dpmt8_sas, servoShedIn_sas,Go_to_vitrine_sas};

void saShed_init(statAndShed *saShed) {
    saShed->status = S0_ps;
    saShed->output = 0;
    saShed->go = 0;
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
    int TEAM = cvs->inputs->team;

    //only if usefull
    double pos[5];
    double x, y, th;
    get_pos(cvs, pos);
    th = pos[2];
    x = pos[0];//+ hlcPF->x_shift * cos(th);
    y = pos[1];//+ hlcPF->x_shift * sin(th);



    switch(saShed->status){
        case S0_sas:
        	if(saShed->go){
        		saShed->status = Dpmt1_sas;
                if (TEAM) set_goal(cvs,2.46,1.51,-2.73);
                else set_goal(cvs,3-2.44,1.55,limit_angle(2.71+M_PI));
        		printf("go to dp1\n");
        		saShed->go = 0;
        	}
            break;

        case Dpmt1_sas:{
    		sendFromHLCPF(cvs,1);
        	if(hlcPF->output){
        		saShed->status = Dpmt2_sas;
                if (TEAM) set_goal(cvs,2.32,1.48,-10);
                else set_goal(cvs,3-2.3,1.5,-10);
            }
        	break;
        }

        case Dpmt2_sas:{
            set_param_prec(cvs);
            sendFromHLCPF(cvs,1);
            //sendFromMLCPF(cvs,.1,-2.71);
            if(hlcPF->output ){//cvs->mp->x < 2.35){
                saShed->status = servoShedOut_sas;
                setChrono(cvs,1);
                printf("setChrono %d\n", checkChrono(cvs));
                printf("go to servoShedout_ps\n");
                teensy_send(cvs, "A");
            }
            break;
        }

        case servoShedOut_sas: {
            // !!!!!
            if (checkChrono(cvs)) {
                saShed->status = Dpmt4_sas;
                if (TEAM) set_goal(cvs,2.2,1.1,-10);
                else set_goal(cvs,3-2.5,.6,-10);                
                printf("go to Dpmt3_ps\n");
            }
            break;
        }
        case Dpmt3_sas:{
            set_param_prec(cvs);
            sendFromHLCPF(cvs,1,1);

            if(hlcPF->output){
                saShed->status = Dpmt4_sas;
                if (TEAM) set_goal(cvs,2.35,.8,-10);
                else set_goal(cvs,3-2.5,.6,-10);
                //if (TEAM) set_goal(cvs,2.1,1.2,-10);
                //else set_goal(cvs,3-2.1,1.2,-10);
                printf("go to Dpmt5_ps\n");
            }
            break;
        }
        case Dpmt4_sas:{
            set_param_large(cvs);
            sendFromHLCPF(cvs,1,1);

            if(hlcPF->output){
                saShed->status = Dpmt5_sas;
                if (TEAM) set_goal(cvs,2.4,.6,-M_PI/4);
                else set_goal(cvs,3-2.5,.6,-10);
                //if (TEAM) set_goal(cvs,2.1,1.2,-10);
                //else set_goal(cvs,3-2.1,1.2,-10);
                printf("go to Dpmt5_ps\n");
            }
            break;
        }
        case Dpmt5_sas:{
            set_param_large(cvs);
            sendFromHLCPF(cvs,1,1);
            if(hlcPF->output){
                saShed->status = Dpmt6_sas;
                if (TEAM) set_goal(cvs,2.5,.5,-M_PI/4);
                else set_goal(cvs,3-2.5,.6,-10);
                //if (TEAM) set_goal(cvs,2.1,1.2,-10);
                //else set_goal(cvs,3-2.1,1.2,-10);
                printf("go to Dpmt6_ps\n");
            }
            break;
        }
        case Dpmt6_sas:{
            set_param_prec(cvs);
            sendFromHLCPF(cvs,1,1);
            if(hlcPF->output){
                saShed->status = servoShedIn_sas;
                setChrono(cvs,1.2);
                teensy_send(cvs, "B");
                printf("go to servoShedIn_sas\n");
                //saShed->output = 1;
            }
            break;
        }
        case servoShedIn_sas: {
            // !!!!!
            if (checkChrono(cvs)) {
                saShed->status = Dpmt7_sas;
                if (TEAM) set_goal(cvs,2.68,.3,-10);
                else set_goal(cvs,3-2.5,.6,-10);
                teensy_send(cvs, "5");
                //if (TEAM) set_goal(cvs,2.1,1.2,-10);
                //else set_goal(cvs,3-2.1,1.2,-10);
                printf("go to Dpmt6_ps\n");
            }

            break;
        }

        case Dpmt7_sas:{
            set_param_prec(cvs);
            sendFromHLCPF(cvs,1,1);
            teensy_recv(cvs);
            if (teensy->switch_F) {
                motors_stop(cvs);
                setChrono(cvs,3);
                teensy_send(cvs, "6");
                teensy->switch_F = 0;
                saShed->status = Wait_for_5_sas;
                printf("go to Dpmt7_ps\n");
            }
            break;
        }
        case Wait_for_5_sas:{
            if (checkChrono(cvs)) {
                saShed->status = Dpmt8_sas;
                if (TEAM) set_goal(cvs,2.5,.5,-10);
                else set_goal(cvs,3-2.5,.6,-10);
                //if (TEAM) set_goal(cvs,2.1,1.2,-10);
                //else set_goal(cvs,3-2.1,1.2,-10);
                printf("go to Dpmt7_ps\n");
            }
            break;
        }

        case Dpmt8_sas:{
            set_param_prec(cvs);
            sendFromHLCPF(cvs,0,1);
            if(hlcPF->output){
                saShed->status = servoShedIn_sas;
                //set_goal(cvs,saShed->x_goals[3],saShed->y_goals[3]);
                saShed->output = 1;            }
            break;
        }


        default:
            printf("Problem default value in FSM\n");
    }

}
