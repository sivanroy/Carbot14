#include "pushShed.h"


#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <cmath> //for sqrt

#include "pushShed.h"



//enum enum {S0_ps,Dpmt1_ps,Dpmt2_ps,Close_ps,Dpmt3_ps,Dpmt4_ps,Dpmt5_ps,Push_ps,Ok_ps,NotOk_ps};

void pushShed_init(pushShed *pshed) {
    pshed->status = S0_ps;
    pshed->output = 0;
    pshed->go = 1;
    int s = 7;
    double x_goalsI[s] = {2.6,2.28,2.2,2.25,2.52,2.7,2.72};
    double y_goalsI[s] = {1.65,1.51,1.12,0.85,0.52,0.35,1.3};
    for (int i=0; i<s;i++) {
    	pshed->x_goals[i] = x_goalsI[i];
    	pshed->y_goals[i] = y_goalsI[i];
    }
    s = 1;
    double thetasI[4] = {-2.6};
    for (int j=0; j<s;j++){
    	pshed->thetas[j] = thetasI[j];
    }


}

void pushShed_launch(ctrlStruct *cvs){
	cvs->pshed->go = 1;
	cvs->pshed->status = S0_ps;
}

void sendFromMainPot(ctrlStruct *cvs,double x_goal,double y_goal){
	get_d2r_data(cvs); 
    main_pot_force(cvs,x_goal,y_goal);
    if(cvs->hlcPF->output) {
    	motors_stop(cvs);
    	return;
    }
    mlcPF_out(cvs, cvs->hlcPF->v_ref, cvs->hlcPF->theta_ref);
    set_commands(cvs, cvs->mlcPF->r_sp_ref, cvs->mlcPF->l_sp_ref);
    send_commands(cvs);
    set_new_position(cvs);
}

void sendFromMLC(ctrlStruct *cvs,double x_goal,double y_goal){
    get_d2r_data(cvs);
    set_speed_ref(cvs,x_goal,y_goal);
    if(cvs->mlc->reach_goal){
        motors_stop(cvs);
        return;
    }
    set_commands(cvs, cvs->mlc->r_sp_ref, cvs->mlc->l_sp_ref);
    send_commands(cvs);
    set_new_position(cvs);
    update_time(cvs);
}


void sendFromMLCPF(ctrlStruct *cvs,double v_ref, double theta_r){
    get_d2r_data(cvs); 
    mlcPF_out(cvs, v_ref, theta_r);
    set_commands(cvs, cvs->mlcPF->r_sp_ref, cvs->mlcPF->l_sp_ref);
    send_commands(cvs);
    set_new_position(cvs);
    update_time(cvs);
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
        	}
            break;

        case Dpmt1_ps:{
            xg = pshed->x_goals[0];
    		yg = pshed->y_goals[0];
    		sendFromMainPot(cvs,xg,yg);
        	if(hlcPF->output){
        		pshed->status = Dpmt2_ps;
        		printf("go to Rotate1\n");
        	}
        	break;
        }

        case Rotate1_ps:{
            sendFromMLCPF(cvs,0,pshed->thetas[0]);
            if( abs(limit_angle(pshed->thetas[0] - mp->th)) < 0.1){
                pshed->status = Dpmt2_ps;
                printf("got to dpmt2\n");
            }
        	break;
        }

        case Dpmt2_ps:{
    		xg = pshed->x_goals[1];
    		yg = pshed->y_goals[1];
    		sendFromMainPot(cvs,xg,yg);
        	if(hlcPF->output){
        		pshed->status = servoShedOut;
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
        	sendFromMainPot(cvs,xg,yg);
        	if(hlcPF->output){
        		pshed->status = Dpmt4_ps;
        		printf("go to dp4\n");
        	}
        	break;
        }

        case Dpmt4_ps:{
            xg = pshed->x_goals[3];
        	yg = pshed->y_goals[3];
        	sendFromMainPot(cvs,xg,yg);
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
            sendFromMainPot(cvs,xg,yg);
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
            sendFromMainPot(cvs,xg,yg);
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

