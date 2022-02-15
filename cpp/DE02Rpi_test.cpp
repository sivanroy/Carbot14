//
// Created by Louis Libert on 15/02/22.
//

#include "DE02Rpi.h"

int main() {

    DE02Rpi DE02Rpi;

    DE02Rpi.init();

    while (1) {
        int count = DE02Rpi.measure(0,1);
        printf("%d\n", count);
    }

    return 0;
}