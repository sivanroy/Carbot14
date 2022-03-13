/*!
 * \file CAN.cpp
 * \brief Class for CAN communication with the motors
 */

#include "CAN.h"

CAN::CAN() {}

void CAN::init()
{

    system("sudo ifconfig can0 down");
    system("sudo ip link set can0 type can bitrate 125000");
    system("sudo ifconfig can0 up");
    usleep(1000);

    system("cansend can0 708#1EFF40"); // LED ON
    system("cansend can0 708#1EFF00"); // LED OFF
    system("cansend can0 708#1CFF80"); // MOTOR L INIT
    system("cansend can0 708#1DFF80"); // MOTOR R INIT
}

void CAN::motor_commands(int r_cmd, int l_cmd)
{
    r_cmd = r_cmd + 35;
    l_cmd = l_cmd + 35;
    std::stringstream r_cmd_stream;
    std::stringstream l_cmd_stream;
    r_cmd_stream <<"cansend can0 708#26FF";
    l_cmd_stream <<"cansend can0 708#25FF";

    if (l_cmd < 16) l_cmd_stream << "0";
    l_cmd_stream << std::hex << l_cmd;
    std::string sl = l_cmd_stream.str();
    const char * cl = sl.c_str();

    if (r_cmd < 16) r_cmd_stream << "0";
    r_cmd_stream << std::hex << r_cmd;
    std::string sr = r_cmd_stream.str();
    const char * cr = sr.c_str();

    printf("cl = %s | cr = %s\n", cl, cr);
    system(cl);
    system(cr);
}

void CAN::stop()
{
    system("cansend can0 708#25FF23");
    system("cansend can0 708#26FF23");
}