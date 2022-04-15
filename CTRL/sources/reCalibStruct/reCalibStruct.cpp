/*!
 * \file reCalibStruct.cpp
 * \brief functions for recalibration
 */

#include "reCalibStruct.h"

using namespace std::chrono;


enum {S0_rec_static, launch_rec_static, firstTry_rec_static, secondTry_rec_static};

void rec_init(reCalibStruct *rec)
{
    rec->rec_flag = 0;
    rec->static_status = S0_rec_static;

    rec->rpl_nTurn = 0;
    rec->rpl_nTurn_set = 0;

    rec->w_limit = 1;

    rec->m = 0;
    rec->M = 1000;

    double map_ref_pts[2][31] = {{0, 0, 0.055, 0.055, 0, 0, 0.45, 0.45, 1.17, 1.17, 1.275, 1.275, 1.425, 1.425, 1.575, 1.575, 1.725, 1.725, 1.83, 1.83, 2.55, 2.55, 3, 3, 2.945, 2.945, 3, 3, 2.49, 0.51, 0},
                            {0.51, 0.675, 0.675, 0.825, 0.825, 2, 2, 1.97, 1.97, 2, 2, 1.945, 1.945, 2, 2, 1.945, 1.945, 2, 2, 1.97, 1.97, 2, 2, 0.825, 0.825, 0.675, 0.675, 0.51, 0, 0, 0.51}};
    double d_line[31];
    double L = 0;
    double x1, x2, y1, y2, dx, dy, d;
    int i, j, n;
    for (i = 0; i < 31; i++) {
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
    for (i = 0; i < 31; i++) {
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
    rec->iter = 0;
    rec->min_delta = 1e-3;

    rec->wall_margin = 0.1;
}

int rec_ICP(ctrlStruct *cvs, IcpPointToPlane *icp)
{
    myPosition *mp = cvs->mp;
    oppPosition *op = cvs->op;
    rplStruct *rpl = cvs->rpl;
    mThreadsStruct *mt = cvs->mt;
    reCalibStruct *rec = cvs->rec;

    int update_flag_rpl;
    pthread_mutex_lock(&(mt->mutex_rpl));
    update_flag_rpl = rpl->update_flag;
    pthread_mutex_unlock(&(mt->mutex_rpl));

    if (update_flag_rpl == 0) return 0;
    printf("recalib start\n");

    double pos[5];
    double x, y, th;
    get_pos(cvs, pos);
    x = pos[0]; y = pos[1]; th = pos[2];

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
        if (!not_wall(cvs, pt_x, pt_y)) { //!not_wall(cvs, pt_x, pt_y)
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
    rec->iter = icp->fit(rec->rpl_p,rec->m,rec->R,t,0.01);
    auto stop = high_resolution_clock::now();
    auto duration = duration_cast<microseconds>(stop - start);
    printf("rec : duration.count() = %lld us\n-------------\n", duration.count());

    double new_x, new_y, new_th;
    pthread_mutex_lock(&(mt->mutex_mp));
    x = mp->x;
    y = mp->y;
    th = mp->th;
    new_x = rec->R.val[0][0]*x + rec->R.val[0][1]*y + t.val[0][0];
    new_y = rec->R.val[1][0]*x + rec->R.val[1][1]*y + t.val[0][1];
    new_th = limit_angle(th + asin(rec->R.val[1][0]));
    if ((new_x > 0 + rec->wall_margin && new_x < 3 - rec->wall_margin) &&
        (new_y > 0 + rec->wall_margin && new_y < 2 - rec->wall_margin)) {
        printf("Recalib pos\n");
        mp->x = new_x;
        mp->y = new_y;
        mp->th = new_th;
    }
    pthread_mutex_unlock(&(mt->mutex_mp));

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

    return 1;
}

int rec_ON(ctrlStruct *cvs)
{
    mThreadsStruct *mt = cvs->mt;
    reCalibStruct *rec = cvs->rec;

    pthread_mutex_lock(&(mt->mutex_rec));
    if (rec->rec_flag == 0) rec->rec_flag = 1;
    pthread_mutex_unlock(&(mt->mutex_rec));

    if (rec->rec_flag) return 1;
    return 0;
}

int rec_static(ctrlStruct *cvs)
{
    rplStruct *rpl = cvs->rpl;
    reCalibStruct *rec = cvs->rec;

    switch (rec->static_status) {
        case S0_rec_static: {
            if (rec->iter == 0)  {
                rec->rpl_nTurn = rpl->nTurns + 2;
                printf("S0_rec_static : try 1\n");
            }
            if (rec->iter == -1) {
                rec->rpl_nTurn = rpl->nTurns + 1;
                printf("S0_rec_static : try 2\n");
            }
            rec->static_status = launch_rec_static;
            return 0;
        }
        case launch_rec_static: {
            if (rpl->nTurns == rec->rpl_nTurn) {
                if (rec_ON(cvs)) {
                    if (rec->iter == 0)  rec->static_status = firstTry_rec_static;
                    if (rec->iter == -1) rec->static_status = secondTry_rec_static;
                }
            }
            return 0;
        }
        case firstTry_rec_static: {
            if (rec->iter > 0) {
                if (rec->iter < rec->max_iter) {
                    rec->iter = 0;
                    rec->static_status = S0_rec_static;
                    printf("rec_static : firstTry\n");
                    return 1;
                } else {
                    rec->iter = -1;
                    rec->static_status = S0_rec_static;
                    return 0;
                }
            }
            return 0;
        }
        case secondTry_rec_static: {
            if (rec->iter > 0) {
                rec->iter = 0;
                printf("rec_static : secondTry\n");
                return 1;
            }
            return 0;
        }
        default:
            return 0;
    }
}