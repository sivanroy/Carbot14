#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <cmath> //for sqrt

#include "statuetteAndShed.h"
#include "FSMs_utils.h"


//angle : opposé + pi

enum {S0_sas,Dpmt1_sas,servoShedOut_sas,Dpmt2_sas,Dpmt3_sas,Dpmt4_sas,Dpmt5_sas,rec_sas,Dpmt_prec_stat,Dpmt6_sas,Wait_for_stat_sas,Dpmt7_sas,Dpmt8_sas,Wait_for_cube_sas,Dpmt9_sas, servoShedIn_sas,Go_to_vitrine_sas};

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
        case S0_sas:{
        	//if(saShed->go){
        		saShed->status = Dpmt1_sas;
                if (TEAM) set_goal(cvs,2.48,1.55,-2.73);//2.47,1.55,-2.75
                else set_goal(cvs,3-2.48,1.55,limit_angle(2.73+M_PI));
        		printf("go to dp1\n");
        		saShed->go = 0;
                //saShed->output = 1;
            //}
            break;
        }

        case Dpmt1_sas:{
            set_param_normal(cvs);
            hlcPF->error = 0.03;
    		sendFromHLCPF(cvs,1,1);
        	if(hlcPF->output){
        		saShed->status = Dpmt2_sas;
                if (TEAM) set_goal(cvs,2.31,1.495,-10); //2.32,1.51
                else set_goal(cvs,0.67,1.51,-10);
                //saShed->output = 1;
            }
        	break;
        }

        case Dpmt2_sas:{
            set_param_normal(cvs);
            hlcPF->error = 0.08;
            //hlcPF->Tau_max = .15;
            //hlcPF->Tau_min = .1;
            //mlcPF->sigma = 0.5;
            sendFromHLCPF(cvs,1,1);
            if(hlcPF->output ){
                saShed->status = servoShedOut_sas;
                setChrono(cvs,0.3);
                printf("setChrono %d\n", checkChrono(cvs));
                printf("go to servoShedout_ps\n");
                teensy_send(cvs, "A");
                //saShed->output = 1;
            }
            break;
        }

        case servoShedOut_sas: {
            // !!!!!
            if (checkChrono(cvs)) {
                saShed->status = Dpmt4_sas;
                if (TEAM) set_goal(cvs,2.2,1.05,-10);
                else set_goal(cvs,3-2.2,1.05,-10);                
                printf("go to Dpmt3_ps\n");
                //saShed->output = 1;
            }
            break;
        }

        case Dpmt3_sas:{
            set_param_prec(cvs);
            sendFromHLCPF(cvs,1,1);
            if(hlcPF->output){
                saShed->status = Dpmt4_sas;
                if (TEAM) set_goal(cvs,2.35,.8,-10);
                else set_goal(cvs,3-2.35,.8,-10);
                printf("go to Dpmt5_ps\n");
                //saShed->output = 1;
            }
            break;

        }
        case Dpmt4_sas:{
            set_param_large(cvs);
            hlcPF->Tau_max = 1;
            hlcPF->Tau_min = 0.2;
            mlcPF->sigma = .8;
            sendFromHLCPF(cvs,1,1);

            if(hlcPF->output){
                saShed->status = Dpmt5_sas;
                if (TEAM) set_goal(cvs,2.42,.6,-M_PI/4);
                else set_goal(cvs,.52,.6,-3*M_PI/4);
                printf("go to Dpmt5_ps\n");
            }
            break;
        }
        case Dpmt5_sas:{
            set_param_normal(cvs);
            hlcPF->Tau_max = 1;
            hlcPF->Tau_min = 0.2;
            mlcPF->sigma = .8;
            sendFromHLCPF(cvs,1,1);
            if(hlcPF->output){
                saShed->status = rec_sas;
                printf("rec START\n");
            }
            break;
        }
        case rec_sas:{
            if (rec_static(cvs)) {
                printf("rec END\n");
                saShed->status = Dpmt_prec_stat;
                if (TEAM) set_goal(cvs, 2.55, .45, -M_PI / 4);
                else set_goal(cvs, .52, .6, -3 * M_PI / 4);
            }
            break;
        }
        case Dpmt_prec_stat:{
            set_param_prec(cvs);
            hlcPF->Tau_max = .15;
            hlcPF->Tau_min = .1;
            mlcPF->sigma = 0.5;
            cvs->mlcPF->Kp_th = 10;
            sendFromHLCPF(cvs,1,1);
            if(hlcPF->output){
                saShed->status = Dpmt6_sas;
                if (TEAM) set_goal(cvs,2.97,0.01,-10);//2.73,.30,-10
                else set_goal(cvs,0.0,.07,-10);
                printf("go to Dpmt6_ps\n");
                teensy_send(cvs, "B");
                teensy_send(cvs, "5");
                //saShed->output = 1;
            }
            break;
        }
        case servoShedIn_sas: {
            // !!!!!
            if (checkChrono(cvs)) {
                saShed->status = Dpmt6_sas;
                if (TEAM) set_goal(cvs,2.98,0.01,-10);//2.73,.30,-10
                else set_goal(cvs,0.0,.07,-10);
                printf("go to Dpmt6_ps\n");

                //teensy_send(cvs, "5");
                //saShed->output = 1;
            }

            break;
        }
        case Dpmt6_sas:{
            set_param_prec(cvs);
            hlcPF->Tau_max = .05;
            hlcPF->Tau_min = .05;
            //mlcPF->sigma = 1;
            //cvs->mlcPF->Kp_th = 10;
            sendFromHLCPF(cvs,1,1);
            if (teensy->switch_F) {
                motors_stop(cvs);
                setChrono(cvs,0.5);
                teensy_send(cvs, "6");
                teensy->switch_F = 0;
                saShed->status = Wait_for_stat_sas;
                arduino_send(cvs,"A");
                teensy_send(cvs, "5");
                //saShed->output = 1;
            }
            break;
        }
        case Wait_for_stat_sas:{
            if (checkChrono(cvs)) {
                saShed->status = Dpmt7_sas;
                if (TEAM) set_goal(cvs,2.57,.53,-M_PI/4);
                else set_goal(cvs,.43,.53,-3*M_PI/4);
                //teensy_send(cvs, "R");
                printf("go to Dpmt7_ps\n");
            }
            break;
        }

        case Dpmt7_sas:{
            set_param_normal(cvs);
            sendFromHLCPF(cvs,0,1);
            if(hlcPF->output){
                saShed->status = Dpmt8_sas;
                if (TEAM) set_goal(cvs,2.77,.34,-10);
                else set_goal(cvs,.23,.34,-10);
                printf("go to Dpmt8_ps\n");
                //saShed->output = 1;
            }
            break;
        }

        case Dpmt8_sas:{
            set_param_prec(cvs);
            hlcPF->Tau_max = .05;
            hlcPF->Tau_min = .05;
            //mlcPF->sigma = 0.5;
            //cvs->mlcPF->Kp_th = 10;
            sendFromHLCPF(cvs,1,1);
            if (teensy->switch_F) {
                motors_stop(cvs);
                setChrono(cvs,0.5);
                teensy_send(cvs, "D");
                teensy->switch_F = 0;
                saShed->status = Wait_for_cube_sas;
                arduino_send(cvs,"A");

                teensy_send(cvs,"5");
                //saShed->output = 1;
            }
            break;
        }
        case Wait_for_cube_sas:{
            if (checkChrono(cvs)) {
                saShed->status = Dpmt9_sas;
                if (TEAM) set_goal(cvs,2.4,.6,-M_PI/2);
                else set_goal(cvs,.6,.6,-M_PI/2);
                //teensy_send(cvs, "R");
                printf("go to Dpmt9_ps\n");
            }
            break;
        }
        case Dpmt9_sas:{
            set_param_normal(cvs);
            sendFromHLCPF(cvs,0,1);
            if(hlcPF->output){
                saShed->output = 1;
            }
            break;
        }



        default:
            printf("Problem default value in FSM\n");
    }

}
