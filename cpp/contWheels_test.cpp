//
// Created by Louis Libert on 15/02/22.
//

#include "controlledWheels.h"
#include "DE02Rpi.h"

int main()
{
    DE02Rpi DE02Rpi;
    DE02Rpi.init();

    controlledWheels cw;
    /*
    double deltat = cw.givedt();
    double wheelDiam = cw.givewheelDia();
    double radPerTickEncod = cw.giveradPerTickEnc();
     */
    double deltat = 0.01;
    double wheelDiam = 0.056;
    double radPerTickEncod = 2*M_PI/1840;

    double speedsRef[3]; double speedRef;
    speedsRef[0] = 0; speedsRef[1] = 0.2; speedsRef[2] = 0.5;

    for (int i; i < 3; i++) {
        speedRef = speedsRef[i];
        cw.setSpeed(speedRef, speedRef);
        printf("init speed %d\n", i);

        for (int j; j < 400; j++) {
            int ticksL = DE02Rpi.measure(1, 1);
            int ticksR = DE02Rpi.measure(1, 0);
            printf("ticksL = %d | ticksR = %d\n", ticksL, ticksR);
            double speedMesL = (-ticksL * (wheelDiam/2) * radPerTickEncod)/deltat;
            double speedMesR = (ticksR * (wheelDiam/2) * radPerTickEncod)/deltat;
            cw.sendV(speedMesL, speedMesR, true);
            int error = usleep(1000000 * deltat);
            if (error == -1) printf("error usleep\n");
        }
    }
    cw.stop();

    return 0;
}

