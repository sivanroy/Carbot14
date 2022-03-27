#ifndef _SELECTPALLETFSM_GR5_H_ // adapt it with the name of this file (header guard)
#define _SELECTPALLETFSM_GR5_H_ // must be the same name as the line before

#include "namespace_ctrl.h"
#include "CtrlStruct_gr5.h"


NAMESPACE_INIT(ctrlGr5);

enum {S0_sp,SelectGoal,RunInt,Run,Ok_sp,NotOk_sp,GoHome,releaseT,Recalibrate};

typedef struct select_pallet_FSM
{
    int status;
    int s;
    int nb_targets;
    int output;
    double beginFSM;
    double timeTillBreak;
    double timeToComeBack;
    int goal;
    double xtargets[7];
    double ytargets[7];
    double xint[7];
    double yint[7];
    int A1ptTargets[4]; //index of the target in x/ytargets
    int A2ptTargets[2];
    int A3ptTargets[1];
    int targetsStatus[7];
    int priority[7];
    int go = 0;
} select_pallet_FSM;

void select_pallet_FSM_init(select_pallet_FSM *spFSM);
void select_pallet_loop(CtrlStruct *cvs);
void set_priority(CtrlStruct *cvs);

NAMESPACE_CLOSE();

#endif // end of header guard
