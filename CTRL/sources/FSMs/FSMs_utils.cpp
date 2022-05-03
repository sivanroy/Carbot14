#include <iostream>
#include <unistd.h>
#include <math.h>
#include <chrono>
#include <stdlib.h>


#include "FSMs_utils.h"


void init_checkBlocked(checkBlocked *checkb) {
    //pas fonctionnel du tout
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
    chro->time1 = 0;
    chro->time2 = 0;
    chro->begin1 = 0;
    chro->begin2 = 0;
    chro->output = 0;
}

void setChrono(ctrlStruct *cvs,double enableTime, int i){
    if(i==0){
        cvs->chro->begin = cvs->inputs->t;
        cvs->chro->time = enableTime;    
    }
    if(i==1){
        cvs->chro->begin1 = cvs->inputs->t;
        cvs->chro->time1 = enableTime;  
    }
    if(i==2){
        cvs->chro->begin2 = cvs->inputs->t;
        cvs->chro->time2 = enableTime;  
    }

    cvs->chro->output = 0;
}

int checkChrono(ctrlStruct *cvs, int i){
    double begin = cvs->chro->begin ;
    if (i==1) begin = cvs->chro->begin1;
    if (i==2) begin = cvs->chro->begin2;
    double enableTime = cvs->chro->time;    
    if(i == 1)enableTime = cvs->chro->time1;
    if(i == 2) enableTime = cvs->chro->time2;
    cvs->chro->output = 0;
    double diff = cvs->inputs->t - begin;
    if(diff>=enableTime) {
        cvs->chro->output = 1;
        return 1;
    }
    return 0;
}

void sendFromHLCPF(ctrlStruct *cvs,int goForward,int noWall,int stopIf, double d_max){
    dyn_obs_set(cvs);
    hlcPF_out(cvs,goForward,noWall);
    if(cvs->hlcPF->output) {
    	motors_stop(cvs);
        printf("stops\n");
    	return;
    } else if (stopIf & cvs->hlcPF->d_opp < d_max){
        motors_stop(cvs);
        set_commands(cvs,0,0);
        //printf("opposant detecté\n");
        return;
    }
    mlcPF_out(cvs, cvs->hlcPF->v_ref, cvs->hlcPF->theta_ref);
    set_commands(cvs, cvs->mlcPF->r_sp_ref, cvs->mlcPF->l_sp_ref);
    send_commands(cvs);
}

void sendFromMLC(ctrlStruct *cvs,double x_goal,double y_goal,int forward){
    dyn_obs_set(cvs);
    set_speed_ref(cvs,x_goal,y_goal,forward);
    if(cvs->mlc->reach_goal){
        motors_stop(cvs);
        return;
    }
    set_commands(cvs, cvs->mlc->r_sp_ref, cvs->mlc->l_sp_ref);
    send_commands(cvs);
}


void sendFromMLCPF(ctrlStruct *cvs,double v_ref, double th_ref){
    dyn_obs_set(cvs);
    mlcPF_out(cvs, v_ref, th_ref);
    set_commands(cvs, cvs->mlcPF->r_sp_ref, cvs->mlcPF->l_sp_ref);
    send_commands(cvs);
}


int check_blocked(ctrlStruct *cvs){
    int pointer = cvs->checkb->pointer;
    int size  = cvs->checkb->size;
    double *values_l = cvs->checkb->values_l;
    double *values_r = cvs->checkb->values_r;

    pointer += 1;
    pointer %= size;
    values_l[pointer] = 0;//cvs->llc->;
    values_r[pointer] = 0;//cvs->llc->;

    double dif = 0;
    for (int i = 0; i < size; i ++) {
        int p = (pointer+i)%size;
        int pp = (pointer+i+1)%size;
        dif += abs(values_l[p]-values_r[pp]);
    }
    if (dif < cvs->checkb->val_err){
        return 1;
    }
    return 0;
}