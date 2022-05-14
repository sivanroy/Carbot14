/*
----------------------------
Welcome to the hlcPF_utils.h
----------------------------
Util functions for the hlcPF
-----------------------------
*/

#ifndef CARBOT14_HIGHLEVELCTRLPF_UTILS_H 
#define CARBOT14_HIGHLEVELCTRLPF_UTILS_H


#include "../ctrlStruct/ctrlStruct.h"
#include "../myPosition/myPosition.h"
#include "../obstacles/obstacles.h"

double tau_compute(ctrlStruct *cvs, int noObst = 0);
int get_partition_map(ctrlStruct *cvs);
double theat_g_compute(double diffx,double diffy);
void compute_incr(ctrlStruct *cvs, double th);

void set_param_normal(ctrlStruct *cvs);
void set_param_prec(ctrlStruct *cvs);
void set_param_large(ctrlStruct *cvs);

#endif 
