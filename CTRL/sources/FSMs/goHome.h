#ifndef _GOHOME_ // adapt it with the name of this file (header guard)
#define _GOHOME_ // must be the same name as the line before

#include "../ctrlStruct/ctrlStruct.h"
#include "../FSMs/FSMs_utils.h"


typedef struct goHome
{   int status;
    int output;
} goHome;

void goHome_init(goHome *ghome);
void goHome_loop(ctrlStruct *cvs);


#endif // end of header guard
