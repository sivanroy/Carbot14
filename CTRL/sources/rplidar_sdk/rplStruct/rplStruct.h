/*!
 * \file rplStruct.h
 * \brief rplidar structure
 */

#ifndef CARBOT14_RPLSTRUCT_H
#define CARBOT14_RPLSTRUCT_H

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <iostream>
#include <unistd.h>
#include <errno.h>
#include <math.h>

#include "../ctrlStruct/ctrlStruct.h"
#include "../sdk/include/rplidar.h"
#include "../sdk/include/rplidar_driver.h"

#ifndef _countof
#define _countof(_Array) (int)(sizeof(_Array) / sizeof(_Array[0]))
#endif

#define MAX_DATA_SIZE 8192


typedef struct rplStruct
{
    rp::standalone::rplidar::RPlidarDriver* lidar;
    rplidar_response_measurement_node_hq_t nodes[MAX_DATA_SIZE];
    size_t count;
    u_result op_result;

    int data_size;
    int nTurns;
    int data_updated;
    double a[MAX_DATA_SIZE];
    double d[MAX_DATA_SIZE];
    double q[MAX_DATA_SIZE];

} rplStruct;

void rpl_init(rplStruct *rpl);
int  rpl_config(rplStruct *rpl);
int  rpl_grabData(ctrlStruct *cvs);
void rpl_stop(ctrlStruct *cvs);

#endif
