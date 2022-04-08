/*!
 * \file reCalibStruct.cpp
 * \brief functions for recalibration
 */

#include "reCalibStruct.h"

using namespace std::chrono;


void rec_init(reCalibStruct *rec)
{
    rec->rec_flag = 0;

    rec->m = 0;
    rec->M = 1000;

    double map_ref_pts[2][15] = {{0, 0, 0.45, 0.45, 1.17, 1.17, 1.83, 1.83, 2.55, 2.55, 3, 3, 2.49, 0.51, 0},
                            {0.51, 2, 2, 1.97, 1.97, 2, 2, 1.97, 1.97, 2, 2, 0.51, 0, 0, 0.51}};
    double d_line[15];
    double L = 0;
    double x1, x2, y1, y2, dx, dy, d;
    int i, j, n;
    for (i = 0; i < 14; i++) {
        x1 = map_ref_pts[0][i];
        x2 = map_ref_pts[0][i+1];
        y1 = map_ref_pts[1][i];
        y2 = map_ref_pts[1][i+1];
        dx = x2 - x1;
        dy = y2 - y1;
        d = sqrt(dx*dx + dy*dy);
        L += d;
        d_line[i] = d;
    }
    int k = 0;
    for (i = 0; i < 14; i++) {
        n = int(rec->M * d_line[i]/L);
        x1 = map_ref_pts[0][i];
        x2 = map_ref_pts[0][i+1];
        y1 = map_ref_pts[1][i];
        y2 = map_ref_pts[1][i+1];
        dx = (x2 - x1)/n;
        dy = (y2 - y1)/n;

        for (j = 0; j < n; j++) {
            rec->map_p[2*k]     = x1 + dx*j;
            rec->map_p[2*k + 1] = y1 + dy*j;
            k++;
        }
    }
    rec->M = k;

    rec->R = Matrix::eye(2);
    rec->max_iter = 10;
    rec->min_delta = 1e-3;
}

int rec_ICP(ctrlStruct *cvs, IcpPointToPlane *icp)
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

    rec->m = 0;

    pthread_mutex_lock(&(mt->mutex_rpl));
    for (i = 0; i < rpl->data_size; i++) {
        a = rpl->a[i];
        d = rpl->d[i];
        pt_or = th - a;
        pt_x = (x + e * cos(th)) + (d * cos(pt_or));//
        pt_y = (y + e * sin(th)) + (d * sin(pt_or));//
        if (!not_wall(cvs, pt_x, pt_y)) {
            rec->rpl_p[2*rec->m]     = pt_x;
            rec->rpl_p[2*rec->m + 1] = pt_y;
            rec->m++;
            fprintf(cvs->icp2_data, "%f,%f\n", pt_x, pt_y);
        }
    }
    rpl->update_flag = 0;
    pthread_mutex_unlock(&(mt->mutex_rpl));

    rec->R = Matrix::eye(2);
    Matrix t(2,1);

    auto start = high_resolution_clock::now();
    icp->fit(rec->rpl_p,rec->m,rec->R,t,0.01);
    auto stop = high_resolution_clock::now();
    auto duration = duration_cast<microseconds>(stop - start);
    printf("rec : duration.count() = %lld us\n-------------\n", duration.count());

    double new_x = rec->R.val[0][0]*x + rec->R.val[0][1]*y + t.val[0][0];
    double new_y = rec->R.val[1][0]*x + rec->R.val[1][1]*y + t.val[0][1];
    double new_th = th + asin(rec->R.val[1][0]);
    fprintf(cvs->rec_data, "%f,%f,%f\n", new_x, new_y, new_th);
    printf("tx = %f | ty = %f |  ", t.val[0][0], t.val[0][1]);
    printf("R00 = %f | R01 = %f\n", rec->R.val[0][0], rec->R.val[0][1]);
    printf("                                  R10 = %f | R11 = %f\n", rec->R.val[1][0], rec->R.val[1][1]);
    printf("new_x = %f | new_y = %f | new_th = %f\n", new_x, new_y, new_th*180/M_PI);

    double Sx;
    double Sy;
    for (i = 0; i < rec->m*2; i+=2) {
        Sx = rec->R.val[0][0] * rec->rpl_p[i] + rec->R.val[0][1] * rec->rpl_p[i+1] + t.val[0][0];
        Sy = rec->R.val[1][0] * rec->rpl_p[i] + rec->R.val[1][1] * rec->rpl_p[i+1] + t.val[1][0];
        fprintf(cvs->icp3_data, "%f,%f\n", Sx, Sy);
    }

    pthread_mutex_lock(&(mt->mutex_rec));
    rec->rec_flag = 0;
    pthread_mutex_unlock(&(mt->mutex_rec));

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