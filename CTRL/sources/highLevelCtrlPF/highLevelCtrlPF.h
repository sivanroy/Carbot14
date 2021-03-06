/*
----------------------------
Welcome to the highLevelCtrlPF.h
----------------------------
Functions and structure defining the high level controller potential 
field path planning (hlcPF)
-----------------------------
*/


#ifndef CARBOT14_HIGHLEVELCTRLPF_H // adapt it with the name of this file (header guard)
#define CARBOT14_HIGHLEVELCTRLPF_H // must be the same name as the line before

#include "../ctrlStruct/ctrlStruct.h"
#include "../myPosition/myPosition.h"
#include "../obstacles/obstacles.h"


typedef struct highLevelCtrlPF
{   //outputs
    int output_main;
    int output ;
    double d ;
    //param physique
    double x_shift ;
    double error ;
    double nearestObst;
    double d_opp;
    //references
    double v_ref ;
    double theta_ref;
    double vx ;
    double vy ;
    int reorientation;
    //Forces
    double F_att[2];
    double F_rep[2];
    //parameters
    double maxF_att ;
    double maxF_rep ;
    double goal[2] ;
    double orientation;
    //repulsive static
    double Eta ; 
    double Rho ;  
    //param dyn obstacle
    double Eta_opp;
    double Rho_opp;
    //attractive
    double d_limit ;
    double Alpha ;
    //tau
    double Tau_max;
    double tau_max_dist ;
    double Tau_min ;
    double tau_min_dist ;
    //reorientation
    double erreurTh;
    double K_th;
    //local minimum
    double Fmin ;
    int flag_min_local;
    double begin_min_local_dodge;
    double goal_local_dodge[2];
    double dodge_incr;

    double Kp_th_reorient;
} highLevelCtrlPF;

void hlcPF_init(highLevelCtrlPF *hlcPF);
void calc_AttractivePotential(ctrlStruct *cvs,double x_goal,double y_goal);
void calc_RepulsivePotential(ctrlStruct *cvs);


/*goForward ->  1 = FORWARD
*               0 = BACKWARD
*              -1 = NO PREF
* orientation   -10 = No Orientation*/
void set_goal(ctrlStruct *cvs, double xgoal, double ygoal, double orientation=-10);
void main_pot_force(ctrlStruct *cvs, double x_goal, double y_goal,int goForward = 1, int noObst = 0);
void hlcPF_out(ctrlStruct *cvs, int goForward = 1,int noObst = 0);

#endif // end of header guard