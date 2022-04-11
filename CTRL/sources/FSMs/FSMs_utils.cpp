#include <iostream>
#include <unistd.h>
#include <math.h>
#include <chrono>
#include <stdlib.h>


#include "FSMs_utils.h"


void sendFromHLCPF(ctrlStruct *cvs,int goForward,int noWall){
	get_d2r_data(cvs);
    dyn_obs_set(cvs);

    hlcPF_out(cvs,goForward,noWall);
    if(cvs->hlcPF->output) {
    	motors_stop(cvs);
        printf("stops\n");
    	return;
    }
    mlcPF_out(cvs, cvs->hlcPF->v_ref, cvs->hlcPF->theta_ref);
    set_commands(cvs, cvs->mlcPF->r_sp_ref, cvs->mlcPF->l_sp_ref);
    send_commands(cvs);
    set_new_position(cvs);
}

void sendFromMLC(ctrlStruct *cvs,double x_goal,double y_goal,int forward){
    get_d2r_data(cvs);
    dyn_obs_set(cvs);
    set_speed_ref(cvs,x_goal,y_goal,forward);
    if(cvs->mlc->reach_goal){
        motors_stop(cvs);
        return;
    }
    set_commands(cvs, cvs->mlc->r_sp_ref, cvs->mlc->l_sp_ref);
    send_commands(cvs);
    set_new_position(cvs);
}


void sendFromMLCPF(ctrlStruct *cvs,double v_ref, double theta_r){
    get_d2r_data(cvs); 
    dyn_obs_set(cvs);
    mlcPF_out(cvs, v_ref, theta_r);
    set_commands(cvs, cvs->mlcPF->r_sp_ref, cvs->mlcPF->l_sp_ref);
    send_commands(cvs);
    set_new_position(cvs);
}