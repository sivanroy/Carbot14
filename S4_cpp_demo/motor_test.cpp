#include <iostream>
#include <unistd.h>
#include "CAN.h"

/*
 * Cmd to run :
 * $ gcc -o exec motor_test.cpp -lwiringPi
 * $ sudo ./exec
 */

int main() {

    int time = 1;
    int speed = 5;

    CAN wheels;

    wheels.init();

    std::cout << "init" << "\n";

    wheels.setSpeeds(speed, speed);
    usleep(1000000 * 2*time);

    wheels.stop();

    return 0;
}
