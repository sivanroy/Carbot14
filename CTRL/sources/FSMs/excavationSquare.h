#ifndef EXCSQUARE_H 
#define EXCSQUARE_H


#include "../ctrlStruct/ctrlStruct.h"


typedef struct excSquares
{
	//v,vx,vy,theta in the orthonormal domain of the map
	int status;
	int output;
	int go;
	double x_goals[10];
	double y_goals[10];
	double thetas[10];
	double forward[10];
} excSquares;

void excSq_init(excSquares *excSq);
void excSq_loop(ctrlStruct *cvs);
void excSq_launch(ctrlStruct *cvs);

#endif // end of header guard