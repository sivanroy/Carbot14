/*!
 * \file reCalibStruct.h
 * \brief functions for recalibration
 */

#ifndef CARBOT14_RECALIBSTRUCT_H
#define CARBOT14_RECALIBSTRUCT_H

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <iostream>
#include <unistd.h>
#include <errno.h>
#include <math.h>
#include <chrono>
#include <pthread.h>
//#include <Eigen/Dense>
//#include "libicp/icpPointToPlane.h"

#include "../ctrlStruct/ctrlStruct.h"


typedef struct reCalibStruct
{
    int M;
    double rpl_p[2][8192];
    double map_p[2][8192];

} reCalibStruct;

void rec_init(reCalibStruct *rec);
int rec_ICP(ctrlStruct *cvs);

#endif
