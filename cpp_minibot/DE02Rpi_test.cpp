//
// Created by Louis Libert on 15/02/22.
//

#include "DE02Rpi.h"

int main() {

    DE02Rpi DE02Rpi;

    DE02Rpi.init();

    while (1) {
        int countL = DE02Rpi.measure(1,1);
        int countR = DE02Rpi.measure(1,0);
        if (countL != 0) printf("L = %d\n", countL);
        if (countR != 0) printf("R = %d\n", countR);
    }

    return 0;
}