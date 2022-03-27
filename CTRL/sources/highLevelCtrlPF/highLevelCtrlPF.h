/*!
 * \file highLevelCtrlPF_gr5.h
 * \brief File description
 */
#ifndef CARBOT14_HIGHLEVELCTRLPF_H // adapt it with the name of this file (header guard)
#define CARBOT14_HIGHLEVELCTRLPF_H // must be the same name as the line before

#include "../ctrlStruct/ctrlStruct.h"
#include "../myPosition/myPosition.h"
#include "../obstacles/obstacles.h"


typedef struct highLevelCtrlPF
{
	int output;
	double d;
	//v,vx,vy,theta in the orthonormal domain of the map
	double v_ref;
	double theta_ref;
	double vx;
	double vy;

	double F_att[2];
	double F_rep[2];

	//Constant for the file path planning
	double Alpha;
	double Eta;
	double Rho;
	double Tau;
	double d_limit;

	double goal[2];

	int flag_min_local;
	double begin_min_local_dodge;
	double goal_local_dodge[2];
	double dodge_incr;
} highLevelCtrlPF;

void hlcPF_init(highLevelCtrlPF *hlcPF);
void calc_AttractivePotential(ctrlStruct *cvs,double x_goal,double y_goal);
void calc_RepulsivePotential(ctrlStruct *cvs);
void main_pot_force(ctrlStruct *cvs,double x_goal,double y_goal);


#endif // end of header guard