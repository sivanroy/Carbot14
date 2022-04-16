#include <iostream>
#include <unistd.h>
#include <math.h>
#include <chrono>
#include <stdlib.h>


#include "FSMs_utils.h"


void init_checkBlocked(checkBlocked *checkb) {
    checkb->size = 200; // !!  à la size < 200
    checkb->pointer = 0;
    checkb->val_err = 0.01;
    for (int i = 0; i< checkb->size; i++) {
        checkb->values_l[i] = -1;
        checkb->values_r[i] = -1;
    }
}

void init_chrono(Chrono* chro){
    chro->begin = 0;
    chro->time = 0;
    chro->output = 0;
}

void setChrono(ctrlStruct *cvs,double enableTime){
    cvs->chro->begin = cvs->inputs->t;
    cvs->chro->time = enableTime;    
    cvs->chro->output = 0;
}

int checkChrono(ctrlStruct *cvs){
    double begin = cvs->chro->begin ;
    double enableTime = cvs->chro->time;    
    cvs->chro->output = 0;
    double diff = cvs->inputs->t - begin;
    if(diff>=enableTime) {
        cvs->chro->output = 1;
        return 1;
    }
    return 0;
}

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


void sendFromMLCPF(ctrlStruct *cvs,double v_ref, double th_ref){
    get_d2r_data(cvs); 
    dyn_obs_set(cvs);
    mlcPF_out(cvs, v_ref, th_ref);
    set_commands(cvs, cvs->mlcPF->r_sp_ref, cvs->mlcPF->l_sp_ref);
    send_commands(cvs);
    set_new_position(cvs);
}


int checkBlocked(ctrlStruct *cvs){
    int pointer = cvs->llc->pointer;
    int size  = cvs->llc->size;
    double *values = cvs->llc->values;
    pointer += 1;
    pointer %= size;
    values_l[pointer] = cvs->llc->l;
    values_r[pointer] = cvs->llc->;

    double dif = 0;
    for (int i = 0; i < size; i ++) {
        int p = (pointer+i)%size;
        int pp = (pointer+i+1)%size;
        dif += abs(values[p]-values[pp]);
    }
    if (dif < cvs->mlc->val_err){
        return 1;
    }
    return 0;
}