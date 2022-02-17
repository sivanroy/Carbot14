//
// Created by Louis Libert on 17/02/22.
//
#include "DE02Rpi.h"
#include "DualMC33926RPi.h"

#include <math.h>

int main() {

    DualMC33926RPi wheels;
    wheels.init();

    DE02Rpi DE02Rpi;
    DE02Rpi.init();

    int uL = 10; int uR = 20;
    wheels.setSpeeds(uL, uR);

    double dt = 0.01;
    double wheelDiam = 0.06;
    double radPerTickEncod = 2*M_PI/1840;
    int i = 0;

    usleep(1000000 * 1);
    while (i < 100) {
        int ticksL = DE02Rpi.measure(1, 1);
        int ticksR = DE02Rpi.measure(1, 0);
        printf("%d : ticksL = %d | ticksR = %d\n", i, ticksL, ticksR);

        double speedMesL = (ticksL * (wheelDiam/2) * radPerTickEncod)/dt;
        double speedMesR = (-ticksR * (wheelDiam/2) * radPerTickEncod)/dt;
        printf("     spMesL = %d | spMesR = %d\n", speedMesL, speedMesR);

        //usleep(1000000 * dt);
        i++;
    }
    return 0;
}

