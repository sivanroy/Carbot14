#include "../ctrlStruct/ctrlStruct.h"
#include "../myPosition/myPosition.h"
#include "../obstacles/obstacles.h"
#include "hlcPF_utils.h"







double tau_compute(ctrlStruct *cvs);
int get_partition_map(ctrlStruct *cvs);
double theat_g_compute(double diffx,double diffy);
void compute_incr(ctrlStruct *cvs, double th);