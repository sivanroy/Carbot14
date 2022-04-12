#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <cmath> //for sqrt
#include <algorithm>    // std::max

#include "hlcPF_utils.h"
#include "highLevelCtrlPF.h" // adapt it with your headers


int get_partition_map(ctrlStruct *cvs){
    highLevelCtrlPF *hlcPF = cvs->hlcPF;
    myPosition *mp = cvs->mp;

    double pos[5];
    double x, y, th;
    get_pos(cvs, pos);
    th = pos[2];
    x = pos[0]+ hlcPF->x_shift * cos(th);
    y = pos[1]+ hlcPF->x_shift * sin(th);

    if(x< 1) {
        if(x>y) {
            return 3;
        } else if (2-y < x) {
            return 1;
        } else {
            return 4;
        }
    } else if (x> 2){
        if(-x+3>y) {
            return 3;
        } else if(x-1<y){
            return 1;
        } else {
            return 2;
        }
    }
    else if (y>1){
        return 1;
    } else {
        return 3;
    } 
    return -1;

}


double theat_g_compute(double diffx,double diffy){
    double theta_g;
    if(diffx != 0) {
        double theta_g = atan(diffy/diffx);
        printf("theta_g computed here : %f\n",theta_g );
        if(diffx < 0) {
            theta_g += 3.1415;
        } 
        if(theta_g > 3.1415) {
            theta_g -= 2*3.1415;
        } else if (theta_g< -3.1415) {
            theta_g += 2*3.1415;
        }
        printf("finally computed here theta_g %f\n",theta_g );
        return theta_g;
    } else {
        printf("else !\n");
        if (diffy > 0) {
            theta_g = 3.1415/2;
            return theta_g;
        } else {
            theta_g = -3.1415/2;
            return theta_g;
        }
    } 
}


void compute_incr(ctrlStruct *cvs, double th) {
    int part = get_partition_map(cvs);
    double incr = 3.1415/2;
    double toincr = 0;
    if(part == 1) {
        if(th>3.1415/2 | th< -3.1415/2){
            toincr += incr;
        } else {
            toincr -= incr;
        }
    } else if (part == 2) {
        if(th> 0) {
            toincr+= incr;
        } else {
            toincr-=incr;
        }
    } else if (part == 3) {
        if(th>3.1415/2 | th< -3.1415/2){
            toincr -= incr;
        } else {
            toincr += incr;
        }
    } else if (part == 4){
        if(th> 0) {
            toincr -= incr;
        } else {
            toincr += incr;
        }
    } else {
        printf("should not happend !! part\n");
    }
    cvs->hlcPF->dodge_incr = toincr;
}


void set_param_normal(ctrlStruct *cvs){
    highLevelCtrlPF *hlcPF = cvs->hlcPF;
    //param physique
    hlcPF->error = 0.02;//0.03; --> 0.01 limit de stabilité (.015 stable)
    //attractive
    double a = 1;
    hlcPF->d_limit = 0.01;
    hlcPF->Alpha = a/hlcPF->d_limit;
    //tau
    hlcPF->Tau_max = 10;
    hlcPF->tau_max_dist = 1.5;
    hlcPF->Tau_min = .01; //0.005;
    hlcPF->tau_min_dist = .01;
    //reorientation
    hlcPF->erreurTh = 0.01;
    //local minimum
    cvs->mlcPF->sigma = .5;
}

void set_param_prec(ctrlStruct *cvs){
    highLevelCtrlPF *hlcPF = cvs->hlcPF;
    //param physique
    hlcPF->error = 0.015;//0.03; --> 0.01 limit de stabilité (.015 stable)
    //attractive
    double a = 1;
    hlcPF->d_limit = 0.01;
    hlcPF->Alpha = a/hlcPF->d_limit;
    //tau
    hlcPF->Tau_max = .2;
    hlcPF->tau_max_dist = .2;
    hlcPF->Tau_min = .05; //0.005;
    hlcPF->tau_min_dist = .01;
    //reorientation
    hlcPF->erreurTh = 0.01;
    //local minimum
    cvs->mlcPF->sigma = .5;
}

//make computation to give a tau dependant of the distance to the opponent (ralentit si trop proche de l'ennemi)

double tau_compute(ctrlStruct *cvs,int noObst) {
    double tau_max = cvs->hlcPF->Tau_max;
    double tau_min = cvs->hlcPF->Tau_min;
    double tau_max_dist = cvs->hlcPF->tau_max_dist;
    double tau_min_dist = cvs->hlcPF->tau_min_dist;

    double a = (tau_min-tau_max)/(tau_min_dist-tau_max_dist);
    double b = tau_min - a*tau_min_dist;

    double x_opp = cvs->obs->obs_dyn_x;
    double y_opp = cvs->obs->obs_dyn_y;
    double d_opp = fmax(sqrt(x_opp*x_opp+y_opp*y_opp),0);

    double d = cvs->hlcPF->d;
    double d_obst = fmax(cvs->hlcPF->nearestObst,0);

    double tau_return = 0;
    double d_return = std::min(d,d_opp);
    if (!noObst) d_return = std::min(d_return,d_obst);

    tau_return = a*d_return+b;

    if (tau_return <= tau_min) tau_return = tau_min;
    else if (tau_return >= tau_max) tau_return = tau_max;

    //distance to opponent
    return tau_return;
}




