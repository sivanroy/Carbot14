/*!
 * \file CAN.h
 * \brief Class for CAN communication with the motors
 */

#ifndef _CAN_H_
#define _CAN_H_

#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <sstream>
#include <string>

class CAN
{
public:
    CAN();
    void init();
    void motor_commands(int r_cmd, int l_cmd);
    void stop();
private:

};


#endif
