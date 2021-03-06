#ifndef CARBOT14_PUSHSHED_H 
#define CARBOT14_PUSHSHED_H


#include "../ctrlStruct/ctrlStruct.h"

enum {S0_ps,Dpmt1_ps,Dpmt2_ps,servoShedOut,servoShedIn,Rotate1_ps,
    Close_ps,Dpmt3_ps,Dpmt4_ps,Dpmt5_ps,Dpmt6_ps,Back,GoHome,Push_ps,Ok_ps,NotOk_ps, Dpmt1_esquive, Close_esquive};


typedef struct pushShed
{
	//v,vx,vy,theta in the orthonormal domain of the map
	int status;
	int output;
	int go;
	double x_goals[10];
	double y_goals[10];
	double thetas[10];
	double forward[10];
} pushShed;

void pushShed_init(pushShed *pshed);
void esquive_loop(ctrlStruct *cvs);
void pushShed_loop(ctrlStruct *cvs);
void pushShed_launch(ctrlStruct *cvs);

#endif // end of header guard