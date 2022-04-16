#ifndef FSM_UTILS_G_R5_H_ // adapt it with the name of this file (header guard)
#define FSM_UTILS_G_R5_H_ // must be the same name as the line before

#include "../ctrlStruct/ctrlStruct.h"


typedef struct checkBlocked {
    //check if blocked
    double values_l[200];
    double values_r[200];
    int size; // !!  à la size < 200
    int pointer;
    double val_err;
} checkBlocked;

typedef struct Chrono {
	double begin;
	double time;
	int output;
} Chrono;

void init_chrono(Chrono* chro);
void init_checkBlocked(checkBlocked *checkb);


void setChrono(ctrlStruct *cvs,double enableTime);
int checkChrono(ctrlStruct *cvs);
int checkBlocked(ctrlStruct *cvs);


void sendFromHLCPF(ctrlStruct *cvs,int goForward=1,int noWall= 0);

void sendFromMLC(ctrlStruct *cvs,double x_goal,double y_goal, int forward = 1);

void sendFromMLCPF(ctrlStruct *cvs,double v_ref, double theta_r);

#endif