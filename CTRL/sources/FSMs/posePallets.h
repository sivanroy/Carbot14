#ifndef POSEPALLETS_H 
#define POSEPALLETS_H


#include "../ctrlStruct/ctrlStruct.h"


typedef struct posePallets
{
	//v,vx,vy,theta in the orthonormal domain of the map
	int status;
	int output;
	int go;
	double x_goals[10];
	double y_goals[10];
	double thetas[10];
	double forward[10];
} posePallets;

void pPallets_init(posePallets *pPallets);
void pPallets_loop(ctrlStruct *cvs);
void pPallets_launch(ctrlStruct *cvs);

#endif // end of header guard