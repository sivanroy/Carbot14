/*!
 * \file rplStruct.cpp
 * \brief rplidar structure
 */

#include "rplStruct.h"


void rpl_init(rplStruct *rpl)
{
    rpl->lidar = rp::standalone::rplidar::RPlidarDriver::CreateDriver();
    if (rpl_config(rpl) == -1) perror("rpl_config() failed\n");

    rpl->e = 0.0505;

    rpl->data_size = 0;
    rpl->nTurns = 0;
    rpl->update_flag = 0;
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
    rplStruct *rpl = cvs->rpl;
    mThreadsStruct *mt = cvs->mt;

    rplidar_response_measurement_node_hq_t nodes[RPL_MAX_DATA_SIZE];
    rpl->count = _countof(nodes);
    rpl->op_result = rpl->lidar->grabScanDataHq(nodes, rpl->count);

    if (IS_OK(rpl->op_result)) {
        rpl->lidar->ascendScanData(nodes, rpl->count);
        rpl->data_size = 0;

        double angle;
        double dist;
        double quality;

        int i;
        pthread_mutex_lock(&(mt->mutex_rpl));
        for (i = 0; i < (int) rpl->count ; ++i){
            angle = (nodes[i].angle_z_q14 * 90.f / (1 << 14)) * M_PI/180;
            dist = (nodes[i].dist_mm_q2/4.0f)/1000;
            quality = nodes[i].quality;
            if (quality > 0 && (dist > 0.18 && dist < 4)) {
                //printf("angle = %f | dist = %f | quality = %f\n", angle, dist, quality);
                rpl->a[rpl->data_size] = angle;
                rpl->d[rpl->data_size] = dist;
                rpl->q[rpl->data_size] = quality;
                rpl->data_size++;
            }
        }
        rpl->nTurns++;
        rpl->update_flag = 1;
        pthread_mutex_unlock(&(mt->mutex_rpl));
        //printf("data_size = %d\n", rpl->data_size);
        printf("nTurns = %d\n", rpl->nTurns);
        return 1;
    }
    else return -1;
}

void rpl_stop(ctrlStruct *cvs)
{
    rplStruct *rpl = cvs->rpl;

    rpl->lidar->stop();
    rp::standalone::rplidar::RPlidarDriver::DisposeDriver(rpl->lidar);
    rpl->lidar = NULL;
}
