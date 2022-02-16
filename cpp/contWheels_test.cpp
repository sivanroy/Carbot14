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
    double deltat = cw.givedt();
    double wheelDiam = cw.givewheelDia();
    double radPerTickEncod = cw.giveradPerTickEnc();

    double speedsRef[3]; double speedRef;
    speedsRef[0] = 0; speedsRef[1] = 0.2; speedsRef[2] = 0.5;

    for (int i; i < 3; i++) {
        speedRef = speedsRef[i];
        cw.setSpeed(speedRef, speedRef);

        for (int j; j < 200; j++) {
            int ticksL = DE02Rpi.measure(1, 1);
            int ticksR = DE02Rpi.measure(1, 0);
            double speedMesL = (-ticksL * (wheelDiam/2) * radPerTickEncod)/deltat;
            double speedMesR = (ticksR * (wheelDiam/2) * radPerTickEncod)/deltat;
            cw.sendV(speedMesL, speedMesR, 1);
        }
    }
    cw.stop();

    return 0;
}

