//
// Created by Louis Libert on 21/02/22.
//

#include "CAN.h"

CAN::CAN() {}

void CAN::init()
{
    system("sudo ifconfig can0 down");
    system("sudo ip link set can0 type can bitrate 125000");
    system("sudo ifconfig can0 up");
    usleep(1000);
    system("cansend can0 708#1EFF40");
    system("cansend can0 708#1EFF00");
    system("cansend can0 708#1CFF80");
    system("cansend can0 708#1DFF80");
}

void CAN::setSpeeds(int cmd_l, int cmd_r)
{
    cmd_l = cmd_l + 35;
    cmd_r = cmd_r + 35;
    std::stringstream cmd_l_stream;
    std::stringstream cmd_r_stream;
    cmd_l_stream <<"cansend can0 708#25FF";
    cmd_r_stream <<"cansend can0 708#26FF";

    if (cmd_l < 16) cmd_l_stream << "0";
    cmd_l_stream << std::hex << cmd_l;
    std::string sl = cmd_l_stream.str();
    const char * cl = sl.c_str();

    if (cmd_r < 16) cmd_r_stream << "0";
    cmd_r_stream << std::hex << cmd_r;
    std::string sr = cmd_r_stream.str();
    const char * cr = sr.c_str();

    system(cl);
    system(cr);
}

void CAN::stop()
{
    system("cansend can0 708#25FF23");
    system("cansend can0 708#26FF23");
}