#ifndef _PALLETFSM_GR5_H_ // adapt it with the name of this file (header guard)
#define _PALLETFSM_GR5_H_ // must be the same name as the line before

#include "namespace_ctrl.h"
#include "CtrlStruct_gr5.h"


NAMESPACE_INIT(ctrlGr5);

enum {S0_p,Dpmt,PrecisDispl, Detected,Wait,Ok,NotOk};

typedef struct pallet_FSM
{
	//v,vx,vy,theta in the orthonormal domain of the map
	int status;
	int output;
	double beginWait;
	double beginWait2;
	int go = 0;
	double goal[2] = {0,0};
	double maxTimeTillBreak;
	double b2;
	int prev_target;

} pallet_FSM;

void pallet_init(pallet_FSM *palFSM);
void pallet_loop(CtrlStruct *cvs);
void pallet_set_goal(CtrlStruct *cvs, double goal[2]);

NAMESPACE_CLOSE();

#endif // end of header guard