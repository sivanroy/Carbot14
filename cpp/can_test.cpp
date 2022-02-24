//
// Created by Louis Libert on 21/02/22.
// Yo

#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
// Include the string library
#include <string>
#include <sstream>

int main()
{
    //0x23 -> 35
    system("cansend can0 708#1CFF80");
    system("cansend can0 708#1DFF80");
    printf("init finished\n");

    system("cansend can0 708#25FF30");
    //char commandL[] = "cansend can0 708#25FF";


    int val = 10;
    std::stringstream commandLStream;
    commandLStream << "cansend can0 708#26FF";
    if (val < 16) {
        commandLStream << "0";
    }
    commandLStream << std::hex << val;
    std::string s = commandLStream.str();
    const char * c = s.c_str();
    system(c);

    printf("%s\n",c );
    usleep(1000000 * 3);
    system("cansend can0 708#25FF23");
    printf("return\n");

    return 0;
}
