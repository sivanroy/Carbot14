#ifndef _STRATEGYFSM_GR5_H_ // adapt it with the name of this file (header guard)
#define _STRATEGYFSM_GR5_H_ // must be the same name as the line before

#include "../ctrlStruct/ctrlStruct.h"
#include "../FSMs/FSMs_utils.h"


typedef struct strategy_FSM
{   int status;
    int output;
    int s;
    int shed;
    
    double timeTillBreak;
    double timeToComeBack;


    int actions[10];
    int timing[10];
    double maxTiming[10];
    double maxt;
    int pt;

} strategy_FSM;

void strategy_FSM_init(strategy_FSM *stratFSM);
void checkIfEnd(ctrlStruct *cvs);
void strategy_loop(ctrlStruct *cvs);
void changeSettings(ctrlStruct *cvs);

#endif // end of header guard
