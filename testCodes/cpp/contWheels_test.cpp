//
// Created by Louis Libert on 15/02/22.
//

#include "controlledWheels.h"
#include "DE02Rpi.h"

using namespace std::chrono;

int main()
{
    DE02Rpi DE02Rpi;
    DE02Rpi.init();

    controlledWheels cw;

    double deltat = 0.01;
    double wheelDiam = 0.06;
    double radPerTickEncod = 2*M_PI/(8192*10);

    long long int dtExec = 0;
    double dL = 0; double dR = 0;

    double speedsRef[4]; double speedRef;
    speedsRef[0] = 0.4; speedsRef[1] = 0.6; speedsRef[2] = -0.2; speedsRef[3] = -0.6;

    for (int i = 0; i < 4; i++) {
        speedRef = speedsRef[i];
        printf("init speed %d\n", i);

        for (int j = 0; j < 200; j++) {
            auto start = high_resolution_clock::now();

            cw.setSpeed(speedRef, speedRef);
            if (i == 2) {
                cw.setSpeed(speedRef, -speedRef);
            }

            int ticksL = DE02Rpi.measure(1, 1);
            int ticksR = DE02Rpi.measure(1, 0);
            double speedMesL = (ticksL * (wheelDiam/2) * radPerTickEncod)/deltat;
            double speedMesR = (-ticksR * (wheelDiam/2) * radPerTickEncod)/deltat;
            dL += speedMesL * deltat;
            dR += speedMesR * deltat;
            printf("***************************************\n");
            printf("sLref = %f | sRref = %f\n", speedRef, speedRef);
            printf("sLmes = %f | sRmes = %f\n", speedMesL, speedMesR);
            printf("ticksL = %d| ticksR = %d\n", ticksL, ticksR);
            cw.sendV(speedMesL, speedMesR, true);
            usleep(1000000 * (0.01-dtExec));

            auto stop = high_resolution_clock::now();
            auto duration = duration_cast<microseconds>(stop - start);
            dtExec += duration.count();
            printf("-> exec time : %lld us\n", duration.count());
        }
    }
    cw.stop();

    printf("\n***************************************\n");
    printf("dL = %f    | dR = %f\n", dL, dR);
    printf("-> exec time : %lld us\n", dtExec);
    printf("***************************************\n");

    return 0;
}

