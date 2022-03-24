//
// Created by Louis Libert on 23/02/22.
//
#include "Lib/DE02Rpi/DE02Rpi.h"
#include "Lib/CAN/CAN.h"

#include <math.h>
#include <chrono>

using namespace std;
using namespace std::chrono;

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
        wheels.setSpeeds(uL, uR);

        countL_enc = DE02Rpi.measure(1,1);
        countR_enc = DE02Rpi.measure(1,0);
        countL_odo = DE02Rpi.measure(0,1);
        countR_odo = DE02Rpi.measure(0,0);

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

