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

void pPalets_init(posePallets *pPalets);
void pPalets_loop(ctrlStruct *cvs);
void pPalets_loop(ctrlStruct *cvs);
void pPalets_launch(ctrlStruct *cvs);

#endif // end of header guard