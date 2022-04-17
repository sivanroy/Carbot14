/*!
 * \file objDetection.h
 * \brief Objects detection in a given position with the lidar
 */

#ifndef CARBOT14_OBJDETECTION_H
#define CARBOT14_OBJDETECTION_H

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <iostream>
#include <unistd.h>
#include <errno.h>
#include <math.h>
#include <chrono>

#include "../ctrlStruct/ctrlStruct.h"

typedef struct objDetection
{
    int od_flag;
    double r_obj;
    double dx_obj;
    double dy_obj;
    int cluster_size_min;

    int distrib_n;
    int pallet_n;

    int status_distrib;
    int rpl_nTurn;

} objDetection;

void od_init(objDetection *od);
int od_ON(ctrlStruct *cvs, int distrib_n);
int od_from_coord(ctrlStruct *cvs, double x_obj, double y_obj, int precise_coord);
int od_distrib_solver(ctrlStruct *cvs);
int od_distrib(ctrlStruct *cvs, int distrib_n, int wait_rpl);


#endif
