/*!
 * \file reCalibStruct.cpp
 * \brief functions for recalibration
 */

#include "reCalibStruct.h"


void rec_init(reCalibStruct *rec)
{
    rec->M = 200;
    //double map_pI[2][M] = {{}, {}};
}

int rec_ICP(ctrlStruct *cvs)
{
    myPosition *mp = cvs->mp;
    rplStruct *rpl = cvs->rpl;
    mThreadsStruct *mt = cvs->mt;
    reCalibStruct *rec = cvs->rec;

    int update_flag_rpl;
    pthread_mutex_lock(&(mt->mutex_rpl));
    update_flag_rpl = rpl->update_flag;
    pthread_mutex_unlock(&(mt->mutex_rpl));

    if (update_flag_rpl == 0) return 0;
    printf("recalib start\n");

    double x, y, th;
    pthread_mutex_lock(&(mt->mutex_mp));
    x = mp->x;
    y = mp->y;
    th = mp->th;
    pthread_mutex_unlock(&(mt->mutex_mp));

    int i, j;
    int M = rec->M;
    double e = rpl->e; // distance between center of wheels and center of rplidar

    double a;
    double d;
    double pt_or;
    double pt_x;
    double pt_y;

    int m = 0;
    pthread_mutex_lock(&(mt->mutex_rpl));
    for (i = 0; i < rpl->data_size; i++) {
        a = rpl->a[i];
        d = rpl->d[i];
        pt_or = th - a;
        pt_x = (x + e * cos(th)) + (d * cos(pt_or));//
        pt_y = (y + e * sin(th)) + (d * sin(pt_or));//
        if (!not_wall(cvs, pt_x, pt_y)) {
            rec->rpl_p[0][m] = pt_x;
            rec->rpl_p[1][m] = pt_y;
            m++;
            fprintf(cvs->rec_data, "%f,%f\n", pt_x, pt_y);
        }
    }
    rpl->update_flag = 0;
    pthread_mutex_unlock(&(mt->mutex_rpl));
    /*
    double closestPoints[m];

    int closest_point_i;
    double min_dist, dist;
    for (i = 0; i < m; i++) {
        min_dist = 99999;
        for (j = 0; j < M; j++) {
            dist = sqrt(pow(rec->rpl_p[0][i] - rec->map_p[0][j], 2) + pow(rec->rpl_p[1][i] - rec->map_p[1][j], 2));
            if (dist < min_dist) {
                min_dist = dist;
                closest_point_i = j;
            }
        }
        closestPoints[i] = j;
    }
    */
    return 1;
}