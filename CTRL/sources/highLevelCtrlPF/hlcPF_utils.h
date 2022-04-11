
#ifndef CARBOT14_HIGHLEVELCTRLPF_UTILS_H // adapt it with the name of this file (header guard)
#define CARBOT14_HIGHLEVELCTRLPF_UTILS_H


#include "../ctrlStruct/ctrlStruct.h"
#include "../myPosition/myPosition.h"
#include "../obstacles/obstacles.h"

//#include "highLevelCtrlPF.h"


double tau_compute(ctrlStruct *cvs, int noObst = 0);
int get_partition_map(ctrlStruct *cvs);
double theat_g_compute(double diffx,double diffy);
void compute_incr(ctrlStruct *cvs, double th);


#endif // end of header guard
