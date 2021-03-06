/*!
 * \file myPosition.cpp
 * \brief Localization of the robot
 */

#include "myPosition.h"
#include "../ctrlStruct/ctrlStruct.h"
#include "../ctrl_IO/ctrlIn.h"
#include "../ctrl_IO/ctrlOut.h"
#include "../lowLevelCtrl/lowLevelCtrl.h"
#include "../midLevelCtrlPF/midLevelCtrlPF.h"


/*! \brief set an angle in the in ]-pi;pi] range
 *
 * \param[in] th angle to limit
 * \return angle limited in ]-pi;pi]
 */
double limit_angle(double th)
{
    while (th <= -M_PI)
    {
        th += 2.0*M_PI;
    }
    while (th > M_PI)
    {
        th -= 2.0*M_PI;
    }
    return th;
}

void mp_init(myPosition *mp)
{
    mp->dt = 0.01;
    mp->R_odo = 0.03;
    mp->b = 0.18;

    mp->x = 0.0;
    mp->y = 0.0;
    mp->th = 0.0;
}

void set_new_position(ctrlStruct *cvs)
{
    // structures declaration
    ctrlIn  *inputs;
    myPosition *mp;

    // structures initialization
    inputs  = cvs->inputs;
    mp  = cvs->mp;

    // measured angular speed of each odo
    double r_sp_mes = inputs->r_sp_mes_odo;
    double l_sp_mes = inputs->l_sp_mes_odo;

    // delta distance 'ds' of each odo
    double ds_r = r_sp_mes * mp->dt * mp->R_odo;
    double ds_l = l_sp_mes * mp->dt * mp->R_odo;

    // delta distance/theta of the robot
    double ds = (ds_r + ds_l)/2;
    double dth = (ds_r - ds_l)/mp->b;

    // delta position of the robot
    double dx = ds * cos(mp->th + dth/2);
    double dy = ds * sin(mp->th + dth/2);

    // new position and orientation of the robot
    mp->x = mp->x + dx;
    mp->y = mp->y + dy;
    mp->th = limit_angle(mp->th + dth);
}