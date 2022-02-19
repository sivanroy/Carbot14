#include <iostream>
#include <unistd.h>
#include "DualMC33926RPi.h"

/*
 * Cmd to run :
 * $ gcc -o exec motor_test.cpp -lwiringPi
 * $ sudo ./exec
 */

int main() {

    int time = 1;
    int speed = 20;

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
    usleep(1000000 * 1*time);

    wheels.stop();

    return 0;
}
