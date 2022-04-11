#ifndef FSM_UTILS_G_R5_H_ // adapt it with the name of this file (header guard)
#define FSM_UTILS_G_R5_H_ // must be the same name as the line before

#include "../ctrlStruct/ctrlStruct.h"


void sendFromHLCPF(ctrlStruct *cvs,int goForward=1,int noWall= 0);

void sendFromMLC(ctrlStruct *cvs,double x_goal,double y_goal, int forward = 1);

void sendFromMLCPF(ctrlStruct *cvs,double v_ref, double theta_r);


#endif