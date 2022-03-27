/*!
 * \file rplStruct.cpp
 * \brief rplidar structure
 */

#include "rplStruct.h"


void rpl_init(rplStruct *rpl)
{
    rpl->lidar = rp::standalone::rplidar::RPlidarDriver::CreateDriver();
    if (rpl_config(rpl) == -1) perror("rpl_config() failed\n");

    rpl->count = _countof(rpl->nodes);
    rpl->op_result = rpl->lidar->grabScanDataHq(rpl->nodes, rpl->count);

    rpl->data_size = 0;
    rpl->nTurns = 0;
    rpl->data_updated = 0;
}

int rpl_config(rplStruct *rpl)
{
    u_result res = rpl->lidar->connect("/dev/ttyUSB0", 115200);
    if (!(IS_OK(res))) return -1;

    std::vector<rp::standalone::rplidar::RplidarScanMode> scanModes;
    rpl->lidar->getAllSupportedScanModes(scanModes);

    rp::standalone::rplidar::RplidarScanMode scanMode;
    rpl->lidar->startMotor();
    rpl->lidar->startScan(0,1);

    return 1;
}

int rpl_grabData(ctrlStruct *cvs)
{
    rplStruct *rpl;
    rpl = cvs->rpl;

    if (IS_OK(rpl->op_result)) {
        rpl->lidar->ascendScanData(rpl->nodes, rpl->count);
        rpl->data_size = 0;

        double angle;
        double dist;
        double quality;

        int i;
        for (i = 0; i < (int) rpl->count ; ++i){
            angle = (rpl->nodes[i].angle_z_q14 * 90.f / (1 << 14)) * M_PI/180;
            dist = rpl->nodes[i].dist_mm_q2/4.0f;
            quality = rpl->nodes[i].quality;
            if (quality > 0 && (dist > 200 && dist < 4000)) {
                //printf("angle = %f | dist = %f | quality = %f\n", angle, dist, quality);
                rpl->a[rpl->data_size] = angle;
                rpl->d[rpl->data_size] = dist;
                rpl->q[rpl->data_size] = quality;
                rpl->data_size++;
            }
        }
        rpl->nTurns++;
        printf("data_size = %d\n", rpl->data_size);
        printf("nTurns = %d\n", rpl->nTurns);
        return 1;
    }
    else return -1;
}

void rpl_stop(ctrlStruct *cvs)
{
    rplStruct *rpl;
    rpl = cvs->rpl;

    rpl->lidar->stop();
    //rpl->lidar->stopMotor();
    rp::standalone::rplidar::RPlidarDriver::DisposeDriver(rpl->lidar);
    rpl->lidar = NULL;
}
