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
    double dt = cw.dt();
    double wheelDia = cw.wheelDia();
    double radPerTickEnc = cw.radPerTickEnc();

    double speedsRef[3]; double speedRef;
    speedsRef[0] = 0; speedsRef[1] = 0.2; speedsRef[2] = 0.5;

    for (int i; i < 3; i++) {
        speedRef = speedsRef[i];
        cw.setSpeed(speedRef, speedRef);

        for (int j; j < 200; j++) {
            int ticksL = DE02Rpi.measure(1, 1);
            int ticksR = DE02Rpi.measure(1, 0);
            double speedMesL = (-ticksL * (wheelDia/2) * radPerTickEnc)/dt;
            double speedMesR = (ticksR * (wheelDia/2) * radPerTickEnc)/dt;
            cw.sendV(speedMesL, speedMesR, 1);
        }
    }
    cw.stop();

    return 0;
}

