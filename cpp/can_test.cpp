//
// Created by Louis Libert on 21/02/22.
//

#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>

int main()
{

    system("cansend can0 708#1CFF80");
    system("cansend can0 708#1DFF80");
    printf("init finished\n");

    system("cansend can0 708#25FF30");



    system("cansend can0 708#26FF30");

    usleep(1000000 * 3);

    system("cansend can0 708#25FF23");
    system("cansend can0 708#26FF23");

    printf("return\n");

    return 0;
}
