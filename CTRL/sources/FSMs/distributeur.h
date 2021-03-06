#ifndef DISTRIBUTEURS_H 
#define DISTRIBUTEURS_H


#include "../ctrlStruct/ctrlStruct.h"


typedef struct distributeurs
{
	//v,vx,vy,theta in the orthonormal domain of the map
	int status;
	int output;
	int go;
	int getHome;
	int getCenter;
	int let;
	int get;
} distributeurs;

void distr_init(distributeurs *distr);
void distr_loop(ctrlStruct *cvs,int center = 0);
void distr_launch(ctrlStruct *cvs);

#endif // end of header guard