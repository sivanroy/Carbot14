#include <iostream>
#include <unistd.h>
#include "DualMC33926RPi.h"

int main() {

    int time = 1;
    int speed = 10;

    DualMC33926RPi wheels;

    wheels.init();

    std::cout << "init" << "\n";

    wheels.setM1Speed(speed);
    usleep(1000000 * time);

    wheels.setM1Speed(-speed);
    usleep(1000000 * time);

    wheels.stop();

    wheels.setM2Speed(speed);
    usleep(1000000 * time);

    wheels.setM2Speed(-speed);
    usleep(1000000 * time);

    wheels.stop();

    wheels.setSpeeds(speed, speed);
    usleep(1000000 * time);

    wheels.setSpeeds(-speed, -speed);
    usleep(1000000 * time);

    wheels.stop();
    usleep(1000000 * time);

    return 0;
}
