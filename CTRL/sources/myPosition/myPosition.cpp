/*!
 * \file myPosition.cpp
 * \brief Localization of the robot
 */

#include "myPosition.h"


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
    mp->dt = 0.003;
    mp->R_odo = 0.023;
    mp->b = 0.2;

    mp->x = 0.0;
    mp->y = 0.0;
    mp->th = 0.0;

    mp->v = 0.0;
    mp->w = 0.0;
}

void set_new_position(ctrlStruct *cvs)
{
    ctrlIn  *inputs = cvs->inputs;
    myPosition *mp = cvs->mp;
    mThreadsStruct *mt = cvs->mt;

    // measured angular speed of each odo
    double r_sp_mes = inputs->r_sp_mes_odo;
    double l_sp_mes = inputs->l_sp_mes_odo;

    // delta distance 'ds' of each odo
    double ds_r = r_sp_mes * mp->dt * mp->R_odo;
    double ds_l = l_sp_mes * mp->dt * mp->R_odo;

    // delta distance/theta of the robot
    double ds = (ds_r + ds_l)/2;
    double dth = (ds_r - ds_l)/mp->b;

    mp->v = ds/mp->dt;
    mp->w = dth/mp->dt;

    // delta position of the robot
    double dx = ds * cos(mp->th + dth/2);
    double dy = ds * sin(mp->th + dth/2);

    // new position and orientation of the robot
    pthread_mutex_lock(&(mt->mutex_mp));
    mp->x = mp->x + dx;
    mp->y = mp->y + dy;
    mp->th = limit_angle(mp->th + dth);
    pthread_mutex_unlock(&(mt->mutex_mp));

    fprintf(cvs->mp_data, "%f,%f,%f,%f,%f,%f\n", mp->x, mp->y, mp->th, mp->w, mp->v, inputs->t);
}

void get_pos(ctrlStruct *cvs, double pos[5])
{
    myPosition *mp = cvs->mp;
    mThreadsStruct *mt = cvs->mt;

    pthread_mutex_lock(&(mt->mutex_mp));
    pos[0] = mp->x;
    pos[1] = mp->y;
    pos[2] = mp->th;
    pos[3] = mp->w;
    pos[4] = mp->v;
    pthread_mutex_unlock(&(mt->mutex_mp));
}