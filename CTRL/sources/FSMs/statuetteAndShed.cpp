#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <cmath> //for sqrt

#include "statuetteAndShed.h"
#include "FSMs_utils.h"


//angle : opposé + pi

enum {S0_sas,Dpmt1_sas,servoShedOut_sas,Dpmt2_sas,Dpmt3_sas,Dpmt4_sas,Dpmt5_sas,Dpmt6_sas,Dpmt7_sas, servoShedIn_sas};

void saShed_init(statAndShed *saShed) {
    saShed->status = S0_ps;
    saShed->output = 0;
    saShed->go = 0;

    int s = 5; //2.2 ;; 1.6 ;; -2.5
    double x_goalsI[s] = {2.44,2.3, 2.5, 2.1,2.5};
    double y_goalsI[s] = {1.55,1.5, .6, 1.2,.45};
    double thetasI[s] = {-2.71,-10,-10,-10,-10}; //s
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
                if (TEAM) set_goal(cvs,2.46,1.52,-2.73);
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
                setChrono(cvs,1.5);
                printf("setChrono %d\n", checkChrono(cvs));
                printf("go to servoShedout_ps\n");
                teensy_send(cvs, "A");
            }
            break;
        }

        case servoShedOut_sas: {
            // !!!!!
            if (checkChrono(cvs)) {
                saShed->status = Dpmt3_sas;
                if (TEAM) set_goal(cvs,2.15,1.1,-10);
                else set_goal(cvs,3-2.5,.6,-10);                
                printf("go to Dpmt3_ps\n");           
            }
            break;
        }

        case Dpmt3_sas:{
            set_param_large(cvs);
            sendFromHLCPF(cvs,1);

            if(hlcPF->output){
                saShed->status = Dpmt4_sas;
                if (TEAM) set_goal(cvs,2.2,.8,-10);
                else set_goal(cvs,3-2.5,.6,-10);   
                //if (TEAM) set_goal(cvs,2.1,1.2,-10);
                //else set_goal(cvs,3-2.1,1.2,-10);   
                printf("go to Dpmt4_ps\n");
            }
            break;
        }

        case Dpmt4_sas:{
            set_param_large(cvs);
            sendFromHLCPF(cvs,1);

            if(hlcPF->output){
                saShed->status = Dpmt5_sas;
                if (TEAM) set_goal(cvs,2.5,.4,-M_PI/4);
                else set_goal(cvs,3-2.5,.6,-10);
                //if (TEAM) set_goal(cvs,2.1,1.2,-10);
                //else set_goal(cvs,3-2.1,1.2,-10);
                printf("go to Dpmt5_ps\n");
                saShed->output = 1;
            }
            break;
        }
        case Dpmt5_sas:{
            set_param_large(cvs);
            sendFromHLCPF(cvs,1,1);
            if(hlcPF->output){
                saShed->status = servoShedIn_sas;
                printf("go to servoShedIn_sas\n");
                saShed->output = 1;
            }
            break;
        }
        case servoShedIn_sas: {
            // !!!!!
            teensy_send(cvs, "B");
            saShed->status = Dpmt6_sas;
            if (TEAM) set_goal(cvs,2.3,.4,-10);
            else set_goal(cvs,3-2.5,.6,-10);
            //if (TEAM) set_goal(cvs,2.1,1.2,-10);
            //else set_goal(cvs,3-2.1,1.2,-10);
            printf("go to Dpmt6_ps\n");
            saShed->output = 1;
            break;
        }

        case Dpmt6_sas:{
            set_param_prec(cvs);
            sendFromHLCPF(cvs,1,1);
            if(hlcPF->output){
                saShed->status = Dpmt7_sas;
                set_goal(cvs,2.52,.43,-M_PI/4);
                printf("go to Dpmt6_ps\n");
                saShed->output = 1;
            }
            break;
        }


        case Dpmt7_sas:{
            set_param_prec(cvs);
            sendFromHLCPF(cvs,1,1);
            if(hlcPF->output){
                saShed->status = servoShedIn_sas;
                //set_goal(cvs,saShed->x_goals[3],saShed->y_goals[3]);
                printf("go to Dpmt2_ps\n");
            }
            break;
        }


        default:
            printf("Problem default value in FSM\n");
    }

}
