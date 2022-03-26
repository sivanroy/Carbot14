/*!
 * \file oppPosition.h
 * \brief Detection of the opponent position
 */

#ifndef CARBOT14_OPPPOSITION_H
#define CARBOT14_OPPPOSITION_H

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <iostream>
#include <unistd.h>
#include <errno.h>
#include <math.h>

#include "../ctrlStruct/ctrlStruct.h"
//#include "../rplidar_sdk/rplStruct/rplStruct.h"


typedef struct oppPosition
{
    int n_opp;
    int cluster_size_min;
    double map_margin;

    double x_op;
    double y_op;

} oppPosition;

void op_init(oppPosition *op);
void get_opp_pos(ctrlStruct *cvs);

#endif
