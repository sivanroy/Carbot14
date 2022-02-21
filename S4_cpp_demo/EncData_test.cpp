//
// Created by Louis Libert on 17/02/22.
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

    double dt = 0.01;
    double wheelDiam = 0.06;
    double radPerTickEncod = 2*M_PI/(2048*4*10);
    int i = 0;

    long long int dtExec = 0;

    double dL = 0; double dR = 0;

    while (i < 50) {
        wheels.setSpeeds(uL, uR);
        auto start = high_resolution_clock::now();

        usleep(1000000 * (0.95*dt));
        int ticksL = DE02Rpi.measure(1, 1);
        int ticksR = DE02Rpi.measure(1, 0);
        printf("%d : ticksL = %d | ticksR = %d\n", i, ticksL, ticksR);

        double speedMesL = (ticksL * (wheelDiam / 2) * radPerTickEncod) / dt;
        double speedMesR = (-ticksR * (wheelDiam / 2) * radPerTickEncod) / dt;
        printf("     spMesL = %f | spMesR = %f\n", speedMesL, speedMesR);

        dL += speedMesL * dt;
        dR += speedMesR * dt;

        i++;

        auto stop = high_resolution_clock::now();
        auto duration = duration_cast<microseconds>(stop - start);
        dtExec += duration.count();
        printf("    -> exec time : %lld us\n", duration.count());
        //cout << duration.count() << endl;

    }
    printf("dL = %f | dR = %f\n", dL, dR);
    printf("-> exec time : %lld us\n", dtExec);
    wheels.stop();
    usleep(1000000 * 0.5);

    return 0;
}

