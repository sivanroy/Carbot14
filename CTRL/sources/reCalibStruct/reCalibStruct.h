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
#include "libicp/icpPointToPlane.h"

#include "../ctrlStruct/ctrlStruct.h"


typedef struct reCalibStruct
{
    int rec_flag; ///< flag set by the main thread : 1 if reCalib needed, 0 otherwise
    double w_limit; ///< omega limit for the reCalibration [rad/s]

    int m; ///< size of the rpl point cloud (rpl_p)
    int M; ///< size of the map point cloud (map_p)
    double rpl_p[2*8192]; ///< rpl point cloud : [x_0, y_0, x_1, y_1, ... , x_m-1, y_m-1]
    double map_p[2*8192]; ///< map point cloud : [x_0, y_0, x_1, y_1, ... , x_M-1, y_M-1]

    Matrix R; ///< rotation matrix of the ICP transformation
    int32_t max_iter; ///< maximum number of iterations of the ICP algorithm
    double min_delta; ///< minimum delta (error) of iterations of the ICP algorithm

    double wall_margin; ///< margin of the walls taken to ensure that the output new position is admissible, in [m]

} reCalibStruct;

void rec_init(reCalibStruct *rec);
int rec_ICP(ctrlStruct *cvs, IcpPointToPlane *icp);
void rec_ON(ctrlStruct *cvs);

#endif
