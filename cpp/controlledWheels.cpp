#include "controlledWheels.h"

#ifndef CONTROLLEDWHEELS_SRC
#define CONTROLLEDWHEELS_SRC

#include <iostream>
#include <unistd.h>
#include "DualMC33926RPi.h"
#include "PidLib/pid.h"

#define WheelP 70
#define WheelI 50
#define WheelD 0
#define WheelMax 50
#define WheelMin -50
#define odoD 0.046
#define wheelD 0.056
#define RadPerTick 0.001

//PID pid = PID(0.1, 100, -100, 0.1, 0.01, 0.5);
/**
int main() {

    PID pid = PID(0.1, 100, -100, 0.1, 0.01, 0.5);

    double val = 20;
    for (int i = 0; i < 100; i++) {
        double inc = pid.calculate(0, val);
        printf("val:% 7.3f inc:% 7.3f\n", val, inc);
        val += inc;
    }

    return 0;
}
**/



#endif