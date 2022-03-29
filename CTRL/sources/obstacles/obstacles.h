/*!
 * \file obstacles_gr5.h
 * \brief File description
 */

#ifndef CARBOT14_OBSTACLES_H // adapt it with the name of this file (header guard)
#define CARBOT14_OBSTACLES_H // must be the same name as the line before

#include "../ctrlStruct/ctrlStruct.h"

#include <stdlib.h>
#include <stdio.h>
#include <string.h>
#include <math.h>


typedef struct obstacles
{
	int sizemax;
	double obs_dyn_x;
	double obs_dyn_y;
	double obs_stat_x[1000];
	double obs_stat_y[1000];
	int size_dyn;
	int size_stat;
} obstacles;

void obs_init(obstacles *obs);
void obs_stat(obstacles *obs);

void lines(obstacles *obs,double xy1[2],double xy2[2],double N);

void dyn_obs_free(ctrlStruct * cvs);
void dyn_obs_set(ctrlStruct *cvs);


#endif