#ifndef CARBOT14_POSESTAT_H 
#define CARBOT14_POSESTAT_H


#include "../ctrlStruct/ctrlStruct.h"


typedef struct poseStatuette
{
	int status;
	int output;
	int go;
} poseStatuette;

void poseStat_init(poseStatuette *poseStat);
void poseStat_loop(ctrlStruct *cvs);
void poseStat_loop(ctrlStruct *cvs);
void poseStat_launch(ctrlStruct *cvs);

#endif // end of header guard