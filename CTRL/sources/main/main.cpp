//
// Created by Louis Libert on 10/03/22.
//
#include <iostream>
#include <unistd.h>
#include <math.h>
#include <chrono>
#include <stdlib.h>

#include "../ctrlStruct/ctrlStruct.h"
#include "../rplidar_sdk/sdk/include/rplidar.h"
#include "../rplidar_sdk/sdk/include/rplidar_driver.h"


#ifndef _countof
#define _countof(_Array) (int)(sizeof(_Array) / sizeof(_Array[0]))
#endif


using namespace std::chrono;


// Variables declaration
u_result op_result;
rp::standalone::rplidar::RPlidarDriver* lidar = rp::standalone::rplidar::RPlidarDriver::CreateDriver();


void stopLidar()
{
    lidar->stop();
    lidar->stopMotor();
    rp::standalone::rplidar::RPlidarDriver::DisposeDriver(lidar);
    lidar = NULL;
}
/*
    create a driver
    start the motor
    start the scan

    @input : none
    @return : void
*/
void lidarConfigure()
{
    u_result res = lidar->connect("/dev/ttyUSB0", 115200);
    if (IS_OK(res)){
        printf("Success \n");
    }
    else{
        printf("Failed to connect to LIDAR\n");
    }
    std::vector<rp::standalone::rplidar::RplidarScanMode> scanModes;
    lidar->getAllSupportedScanModes(scanModes);

    rp::standalone::rplidar::RplidarScanMode scanMode;
    lidar->startMotor();
    lidar->startScan(0,1);
}
/*
int main() {

    CAN wheels;
    wheels.init();

    DE02Rpi DE02Rpi;
    DE02Rpi.init();

    int uL = 10; int uR = 10;

    long long int dt = 10000;
    //double wheelDiam = 0.06;
    //double radPerTickEncod = 2*M_PI/(2048*4*10);
    int i = 0;

    long long int dtExecMax = 0;
    long long int dtExec = 0;

    int countR_enc;
    int countL_enc;
    int countR_odo;
    int countL_odo;

    while (i < 100) {
        auto start = high_resolution_clock::now();
        wheels.motor_commands(uR, uL);

        countL_enc = DE02Rpi.enc_measure(1,1);
        countR_enc = DE02Rpi.enc_measure(1,0);
        countL_odo = DE02Rpi.enc_measure(0,1);
        countR_odo = DE02Rpi.enc_measure(0,0);

        if (abs(countL_enc) > 10) printf("L_enc = %d\n", countL_enc);
        if (abs(countR_enc) > 10) printf("R_enc = %d\n", countR_enc);
        if (abs(countL_odo) > 10) printf("L_odo = %d\n", countL_odo);
        if (abs(countR_odo) > 10) printf("R_odo = %d\n", countR_odo);

        i++;

        auto stop = high_resolution_clock::now();
        auto duration = duration_cast<microseconds>(stop - start);
        dtExec = duration.count();

        dtExecMax += dt;
        printf("    -> exec time : %lld us\n", dtExec);

        if (dtExec < dt) usleep(dt - dtExec);
    }
    printf("dtExecMax = %f s\n", dtExecMax*1e-6);
    wheels.stop();
    wheels.freeCAN();

    return 0;
}


int main()
{
    lidarConfigure();

    rplidar_response_measurement_node_hq_t nodes[8192];
    size_t   count = _countof(nodes);
    op_result = lidar->grabScanDataHq(nodes, count);

    long long int t = 0;
    while (t < 2000000) {
        auto start = high_resolution_clock::now();

        if (IS_OK(op_result)) {
            lidar->ascendScanData(nodes, count);
            for (int pos = 0; pos < (int)count ; ++pos){
                double angle = nodes[pos].angle_z_q14 * 90.f / (1 << 14);
                double dist = nodes[pos].dist_mm_q2/4.0f;
                double quality = nodes[pos].quality;
                if (dist < 200 && quality > 0) {
                    printf("angle = %f | dist = %f | quality = %f\n", angle, dist, quality);
                }
            }
        }
        auto stop = high_resolution_clock::now();
        auto duration = duration_cast<microseconds>(stop - start);
        printf("*********************\nExec time : %lld\n*********************\n", duration.count());
        t += duration.count();
    }
    stopLidar();


    return 0;
}
*/
int main()
{
    ctrlStruct *cvs;
    cvs = cvs_init();
    printf("cvs init : end\n");

    // variables declaration
    ctrlIn  *inputs;
    ctrlOut *outputs;
    lowLevelCtrl *llc;
    myPosition *mp;
    midLevelCtrlPF *mlcPF;
    double dt;

    // variables initialization
    inputs  = cvs->inputs;
    outputs = cvs->outputs;
    llc  = cvs->llc;
    mp = cvs->mp;
    mlcPF = cvs->mlcPF;
    dt = inputs->dt;

    int cmdON = 1;
    int llcON = 0;

    if (cmdON) {
        int r_cmd = 0;
        int l_cmd = 0;

        while (inputs->t < 1.5) {
            auto start = high_resolution_clock::now();

            get_d2r_data(cvs); // ctrlIn

            printf("r_sp_mes_enc = %f | l_sp_mes_enc = %f\n", inputs->r_sp_mes_enc, inputs->l_sp_mes_enc);
            printf("r_sp_mes_odo = %f | l_sp_mes_odo = %f\n", inputs->r_sp_mes_odo, inputs->l_sp_mes_odo);

            if (inputs->t >= 0 && inputs->t < 1.5) {
                r_cmd = 10;
                l_cmd = 10;
            }

            else if (inputs->t >= 1.5 && inputs->t < 3) {
                r_cmd = 10;
                l_cmd = 10;
            }
                /*
                else if (inputs->t >= 4 && inputs->t < 6) {
                    r_cmd = -10;
                    l_cmd = 10;
                }
                */
            else {
                r_cmd = 0;
                l_cmd = 0;
            }
            outputs->r_cmd = r_cmd;
            outputs->l_cmd = l_cmd;
            send_commands(cvs); // ctrlOut

            fprintf(cvs->llc_data, "%f,%d,%d,%f,%f\n", inputs->t, r_cmd, l_cmd, inputs->r_sp_mes_enc, inputs->l_sp_mes_enc);

            update_time(cvs);
            auto stop = high_resolution_clock::now();
            auto duration = duration_cast<microseconds>(stop - start);
            printf("duration.count() = %lld us | t = %f\n-------------\n", duration.count(), inputs->t);

            usleep(dt * 1000000 - duration.count());
        }
    }
    if (llcON) {
        double r_sp_ref = 0.0;
        double l_sp_ref = 0.0;

        while (inputs->t < 1.5) {
            auto start = high_resolution_clock::now();

            get_d2r_data(cvs); // ctrlIn

            printf("r_sp_mes_enc = %f | l_sp_mes_enc = %f\n", inputs->r_sp_mes_enc, inputs->l_sp_mes_enc);
            printf("r_sp_mes_odo = %f | l_sp_mes_odo = %f\n", inputs->r_sp_mes_odo, inputs->l_sp_mes_odo);

            if (inputs->t >= 0 && inputs->t < 1.5) {
                r_sp_ref = 2;
                l_sp_ref = 2;
            }

            else if (inputs->t >= 1.5 && inputs->t < 3) {
                r_sp_ref = 10;
                l_sp_ref = 10;
            }
                /*
                else if (inputs->t >= 4 && inputs->t < 6) {
                    r_cmd = -10;
                    l_cmd = 10;
                }
                */
            else {
                r_sp_ref = 0;
                l_sp_ref = 0;
            }
            set_commands(cvs, r_sp_ref, l_sp_ref);
            printf("cmd_r = %d | cmd_l = %d\n", outputs->r_cmd, outputs->l_cmd);
            send_commands(cvs);

            fprintf(cvs->llc_data, "%f,%f,%f,%f,%f\n", inputs->t, r_sp_ref, l_sp_ref, inputs->r_sp_mes_enc, inputs->l_sp_mes_enc);

            update_time(cvs);
            auto stop = high_resolution_clock::now();
            auto duration = duration_cast<microseconds>(stop - start);
            printf("duration.count() = %lld us | t = %f\n-------------\n", duration.count(), inputs->t);

            usleep(dt * 1000000 - duration.count());
        }
    }

    motors_stop(cvs);
    cvs_free(cvs);

    return 0;
}
