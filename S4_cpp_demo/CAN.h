//
// Created by Louis Libert on 21/02/22.
//

#ifndef CARBOT14_CAN_H
#define CARBOT14_CAN_H

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
    void setSpeeds(int cmd_l, int cmd_r);
    void stop();
private:

};


#endif //CARBOT14_CAN_H
