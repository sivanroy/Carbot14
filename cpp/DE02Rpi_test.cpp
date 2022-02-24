//
// Created by Louis Libert on 15/02/22.
//

#include "DE02Rpi.h"

#include <chrono>

using namespace std;
using namespace std::chrono;

int main() {

    DE02Rpi DE02Rpi;
    DE02Rpi.init();

    long long int dt = 10000;

    long long int dtExec = 0;
    long long int dtExecMax = 0;
    int ticksLmax = 0; int ticksRmax = 0;

    int i = 0;
    while (i < 500) {
        auto start = high_resolution_clock::now();

        int countL = DE02Rpi.measure(1,1);
        int countR = DE02Rpi.measure(1,0);

        if (countL != 0) printf("L = %d\n", countL);
        if (countR != 0) printf("R = %d\n", countR);
        //printf("L = %d\n", countL);
        auto stop = high_resolution_clock::now();
        auto duration = duration_cast<microseconds>(stop - start);
        dtExec = duration.count();
        printf("dt-dtExec = %lld\n", dt - dtExec);

        ticksLmax += countL;
        ticksRmax += countR;

        i++;
        dtExecMax += dt;
        usleep(dt - dtExec);

    }
    printf("ticksLmax = %d | ticksRmax = %d\n", ticksLmax, ticksRmax);

    return 0;
}