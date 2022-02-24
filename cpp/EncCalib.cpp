//
// Created by Louis Libert on 23/02/22.
//
#include "DE02Rpi.h"
#include "CAN.h"

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

    int ticksLmax = 0; int ticksRmax = 0;

    while (i < 100) {
        wheels.setSpeeds(uL, uR);
        auto start = high_resolution_clock::now();


        int ticksL = DE02Rpi.measure(1, 1);
        int ticksR = DE02Rpi.measure(1, 0);
        ticksLmax += ticksL;
        ticksRmax += ticksR;
        printf("%d : ticksL = %d | ticksR = %d\n", i, ticksL, ticksR);

        i++;

        auto stop = high_resolution_clock::now();
        auto duration = duration_cast<microseconds>(stop - start);
        dtExec = duration.count();

        usleep(dt - dtExec);
        dtExecMax += dt;
        printf("    -> exec time : %lld us\n", dt - dtExec);

    }
    printf("ticksLmax = %d | ticksRmax = %d\n", ticksLmax, ticksRmax);
    printf("-> exec time : %lld us\n", dtExecMax);
    wheels.stop();

    return 0;
}

