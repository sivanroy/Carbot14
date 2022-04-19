#ifndef CARBOT14_STATANDSH_H 
#define CARBOT14_STATANDSH_H


#include "../ctrlStruct/ctrlStruct.h"


typedef struct statAndShed
{
	//v,vx,vy,theta in the orthonormal domain of the map
	int status;
	int output;
	int go;
} statAndShed;

void saShed_init(statAndShed *saShed);
void saShed_loop(ctrlStruct *cvs);
void saShed_loop(ctrlStruct *cvs);
void saShed_launch(ctrlStruct *cvs);

#endif // end of header guard