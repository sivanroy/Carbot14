/*!
 * \file rplStruct.cpp
 * \brief rplidar structure
 */

#include "rplStruct.h"


void rpl_init(rplStruct *rpl)
{
    rpl->lidar = rp::standalone::rplidar::RPlidarDriver::CreateDriver();

    if (rpl_config(rpl) == -1) perror("rpl_config() failed");
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

void rpl_stop(rplStruct *rpl)
{
    rpl->lidar->stop();
    rpl->lidar->stopMotor();
    rp::standalone::rplidar::RPlidarDriver::DisposeDriver(rpl->lidar);
    rpl->lidar = NULL;
}
