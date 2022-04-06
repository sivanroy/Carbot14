#ifndef _STRATEGYFSM_GR5_H_ // adapt it with the name of this file (header guard)
#define _STRATEGYFSM_GR5_H_ // must be the same name as the line before

#include "../ctrlStruct/ctrlStruct.h"


typedef struct strategy_FSM
{   int status;
    int output;
    int s;

    double timeTillBreak;
    double timeToComeBack;

    int actions[10];

} strategy_FSM;

void strategy_FSM_init(strategy_FSM *stratFSM);
void strategy_loop(ctrlStruct *cvs);


#endif // end of header guard
