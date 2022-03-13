//
// Created by Louis Libert on 15/02/22.
//

#include "Lib/DE02Rpi/DE02Rpi.h"

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
    while (i < 10000) {
        auto start = high_resolution_clock::now();

        int countL_enc = DE02Rpi.measure(1,1);
        int countR_enc = DE02Rpi.measure(1,0);

        int countL_odo = DE02Rpi.measure(1,1);
        int countR_odo = DE02Rpi.measure(1,0);

        if (countL_enc != 0) printf("L_enc = %d\n", countL_enc);
        if (countR_enc != 0) printf("R_enc = %d\n", countR_enc);
        if (countL_odo != 0) printf("L_odo = %d\n", countL_odo);
        if (countR_odo != 0) printf("R_odo = %d\n", countR_odo);
        //printf("L = %d\n", countL);
        auto stop = high_resolution_clock::now();
        auto duration = duration_cast<microseconds>(stop - start);
        dtExec = duration.count();
        printf("dt-dtExec = %lld\n---------------------------\n", dt - dtExec);

        ticksLmax += countL;
        ticksRmax += countR;

        i++;
        dtExecMax += dt;
        usleep(dt - dtExec);

    }
    printf("ticksLmax = %d | ticksRmax = %d\n", ticksLmax, ticksRmax);

    return 0;
}