#ifndef POSEPALLETS_H 
#define POSEPALLETS_H


#include "../ctrlStruct/ctrlStruct.h"


typedef struct posePallets
{
	//v,vx,vy,theta in the orthonormal domain of the map
	int status;
	int output;
	int go;
	int low;
	int high;
} posePallets;

void pPallets_init(posePallets *pPallets);
void pPallets_loop(ctrlStruct *cvs,int high = 0);
void pPallets_launch(ctrlStruct *cvs);

#endif // end of header guard