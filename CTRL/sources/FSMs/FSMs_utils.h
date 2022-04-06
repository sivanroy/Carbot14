#ifndef _FSM_UTILS_G_R5_H_ // adapt it with the name of this file (header guard)
#define _FSM_UTILS_G_R5_H_ // must be the same name as the line before

#include "../ctrlStruct/ctrlStruct.h"


void sendFromMainPot(ctrlStruct *cvs,int goForward=1, double x_goal=0,double y_goal=0);

void sendFromHLCPF(ctrlStruct *cvs,int goForward=1);

void sendFromMLC(ctrlStruct *cvs,double x_goal,double y_goal);

void sendFromMLCPF(ctrlStruct *cvs,double v_ref, double theta_r);


#endif