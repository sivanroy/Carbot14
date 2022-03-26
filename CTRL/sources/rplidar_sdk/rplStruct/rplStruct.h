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

#define RPL_DATA_SIZE 8192


typedef struct rplStruct
{
    u_result op_result;
    rp::standalone::rplidar::RPlidarDriver* lidar;

} rplStruct;

void rpl_init(rplStruct *rpl);
int  rpl_config(rplStruct *rpl);

void rpl_stop(rplStruct *rpl);

#endif
