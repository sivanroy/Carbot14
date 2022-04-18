/*!
 * \file objDetection.cpp
 * \brief Objects detection in a given position with the lidar
 */

#include "objDetection.h"


enum {S0_od_distrib, launch_od_distrib, receive_od_distrib};

void od_init(objDetection *od)
{
    od->od_flag = 0;
    od->r_obj = 0.01;
    od->dx_obj = 0.01;
    od->dy_obj = 0.01;
    od->cluster_size_min = 3;

    od->distrib_n = -1;
    od->pallet_n = 0;

    od->status_distrib = S0_od_distrib;
    od->rpl_nTurn = 0;
}

int od_ON(ctrlStruct *cvs, int distrib_n)
{
    mThreadsStruct *mt = cvs->mt;
    objDetection *od = cvs->od;

    int flag = 0;
    pthread_mutex_lock(&(mt->mutex_od));
    if (od->od_flag == 0) {
        od->od_flag = 1;
        flag = 1;
        od->distrib_n = distrib_n;
        od->pallet_n = -10;
    }
    pthread_mutex_unlock(&(mt->mutex_od));

    return flag;
}

int od_from_coord(ctrlStruct *cvs, double x_obj, double y_obj, int precise_coord)
{
    rplStruct *rpl = cvs->rpl;
    mThreadsStruct *mt = cvs->mt;
    objDetection *od = cvs->od;

    if (precise_coord) { // coord y needs to be precise
        od->dx_obj = 0.07;
        od->dy_obj = 0.012;
    }
    else {              // coord x needs to be precise
        od->dx_obj = 0.012;
        od->dy_obj = 0.07;
    }

    double pos[5];
    double x, y, th;
    get_pos(cvs, pos);
    x = pos[0]; y = pos[1]; th = pos[2];
    double e = rpl->e; // distance between center of wheels and center of rplidar

    double a;
    double d;
    double pt_or;
    double pt_x;
    double pt_y;

    int size_od = 0;
    double x_od[8192];
    double y_od[8192];

    int i;
    pthread_mutex_lock(&(mt->mutex_rpl));
    for (i = 0; i < rpl->data_size; i++) {
        a = rpl->a[i];
        d = rpl->d[i];
        pt_or = th - a;
        pt_x = (x + e * cos(th)) + (d * cos(pt_or));
        pt_y = (y + e * sin(th)) + (d * sin(pt_or));
        x_od[size_od] = pt_x;
        y_od[size_od] = pt_y;
        size_od++;
    }
    pthread_mutex_unlock(&(mt->mutex_rpl));

    int match_pts_size = 0;
    for (i = 0; i < size_od; i++) {
        if (x_od[i] >= x_obj - od->dx_obj && x_od[i] <= x_obj + od->dx_obj) {
            if (y_od[i] >= y_obj - od->dy_obj && y_od[i] <= y_obj + od->dy_obj) {
                match_pts_size++;
            }
        }
    }
    printf("match_pts_size = %d\n", match_pts_size);
    if (match_pts_size >= od->cluster_size_min) return 1;
    return 0;
}

int od_distrib_solver(ctrlStruct *cvs)
{
    ctrlIn *inputs = cvs->inputs;
    rplStruct *rpl = cvs->rpl;
    mThreadsStruct *mt = cvs->mt;
    objDetection *od = cvs->od;
    int distrib_n = od->distrib_n;

    double pos[5];
    double x, y, th;
    get_pos(cvs, pos);
    x = pos[0]; y = pos[1]; th = pos[2];
    double e = rpl->e; // distance between center of wheels and center of rplidar

    double a;
    double d;
    double pt_or;
    double pt_x;
    double pt_y;

    int size_od = 0;
    double x_od[8192];
    double y_od[8192];

    int i, j;
    pthread_mutex_lock(&(mt->mutex_rpl));
    for (i = 0; i < rpl->data_size; i++) {
        a = rpl->a[i];
        d = rpl->d[i];
        pt_or = th - a;
        pt_x = (x + e * cos(th)) + (d * cos(pt_or));
        pt_y = (y + e * sin(th)) + (d * sin(pt_or));
        x_od[size_od] = pt_x;
        y_od[size_od] = pt_y;
        fprintf(cvs->od_data, "%f,%f\n", pt_x, pt_y);
        size_od++;
    }
    pthread_mutex_unlock(&(mt->mutex_rpl));

    double pallets_x[4];
    double pallets_y[4];
    double x_pallet, y_pallet;
    int precise_coord;
    if (inputs->team == 0) { // purple team
        if (distrib_n == 0) {
            x_pallet = 3 - 0.055;
            y_pallet = 0.75;
            precise_coord = 0;
            for (j = 0; j < 3; j++) {
                pallets_x[j] = x_pallet + j*0.017;
                pallets_y[j] = y_pallet;
            }
            pallets_x[3] = 3;
            pallets_y[3] = y_pallet;
        }
        else if (distrib_n == 1) {
            x_pallet = 1.65;
            y_pallet = 2 - 0.055;
            precise_coord = 1;
            for (j = 0; j < 3; j++) {
                pallets_x[j] = x_pallet;
                pallets_y[j] = y_pallet + j*0.017;
            }
            pallets_x[3] = x_pallet;
            pallets_y[3] = 2;
        }
        else if (distrib_n == 2) {
            x_pallet = 1.35;
            y_pallet = 2 - 0.055;
            precise_coord = 1;
            for (j = 0; j < 3; j++) {
                pallets_x[j] = x_pallet;
                pallets_y[j] = y_pallet + j*0.017;
            }
            pallets_x[3] = x_pallet;
            pallets_y[3] = 2;
        }
    }
    else {
        if (distrib_n == 0) {
            x_pallet = 0.055;
            y_pallet = 0.75;
            precise_coord = 0;
            for (j = 0; j < 3; j++) {
                pallets_x[j] = x_pallet - j*0.017;
                pallets_y[j] = y_pallet;
            }
            pallets_x[3] = 0;
            pallets_y[3] = y_pallet;
        }
        else if (distrib_n == 1) {
            x_pallet = 1.35;
            y_pallet = 2 - 0.055;
            precise_coord = 1;
            for (j = 0; j < 3; j++) {
                pallets_x[j] = x_pallet;
                pallets_y[j] = y_pallet + j*0.017;
            }
            pallets_x[3] = x_pallet;
            pallets_y[3] = 2;
        }
        else if (distrib_n == 2) {
            x_pallet = 1.65;
            y_pallet = 2 - 0.055;
            precise_coord = 1;
            for (j = 0; j < 3; j++) {
                pallets_x[j] = x_pallet;
                pallets_y[j] = y_pallet + j*0.017;
            }
            pallets_x[3] = x_pallet;
            pallets_y[3] = 2;
        }
    }
    if (precise_coord) { // coord y needs to be precise
        od->dx_obj = 0.075;
        od->dy_obj = 0.012;
    }
    else {              // coord x needs to be precise
        od->dx_obj = 0.0145;
        od->dy_obj = 0.075;
    }

    int match_pts_size[4] = {0, 0, 0, 0};
    for (i = 0; i < size_od; i++) {
        for (j = 0; j < 4; j++) {
            if (precise_coord) {
                if (x_od[i] >= pallets_x[j] - od->dx_obj && x_od[i] <= pallets_x[j] + od->dx_obj) {
                    if (y_od[i] >= pallets_y[j] - od->dy_obj && y_od[i] <= pallets_y[j] + 0.017 - od->dy_obj) {
                        match_pts_size[j]++;
                    }
                }
            }
            else {
                if (x_od[i] >= pallets_x[j] - od->dx_obj && x_od[i] <= pallets_x[j] + 0.017 - od->dx_obj) {
                    if (y_od[i] >= pallets_y[j] - od->dy_obj && y_od[i] <= pallets_y[j] + od->dy_obj) {
                        match_pts_size[j]++;
                    }
                }
            }

        }
    }
    int pallet_n = 0;
    int max_match_pts_size = 0;
    for (j = 0; j < 4; j++) {
        if (match_pts_size[j] > max_match_pts_size && match_pts_size[j] >= od->cluster_size_min) pallet_n = j + 1;
    }
    if (pallet_n == 4) pallet_n = -1;

    pthread_mutex_lock(&(mt->mutex_od));
    od->pallet_n = pallet_n;
    od->od_flag = 0;
    pthread_mutex_unlock(&(mt->mutex_od));

    return pallet_n;
}
/*!
 *
 * @param cvs
 * @param distrib_n : distrib that we want to check (0 : ours | 1 : common in our side | 2 : common in opp side)
 * @param wait_rpl : 1 if we want to wait for new lidar data, 0 otherwise
 * @return -1 if we do not detect any pallet (we detect the wall)
 *          0 if fail (we detect nothing)
 *          1 if we detect the first pallet (there are 3 pallets in the distrib)
 *          2 if we detect the second pallet (there are 2 pallets in the distrib)
 *          3 if we detect the third pallet (there is 1 pallet in the distrib)
 */
int od_distrib(ctrlStruct *cvs, int distrib_n, int wait_rpl)
{
    rplStruct *rpl = cvs->rpl;
    mThreadsStruct *mt = cvs->mt;
    objDetection *od = cvs->od;

    pthread_mutex_lock(&(mt->mutex_od));
    int pallet_n = od->pallet_n;
    pthread_mutex_unlock(&(mt->mutex_od));

    switch (od->status_distrib) {
        case S0_od_distrib: {
            od->rpl_nTurn = rpl->nTurns + 2;
            od->status_distrib = launch_od_distrib;
            return -10;
        }
        case launch_od_distrib: {
            if (!wait_rpl) {
                if (od_ON(cvs, distrib_n)) {
                    od->status_distrib = receive_od_distrib;
                    return -10;
                }
            }
            else {
                if (rpl->nTurns == od->rpl_nTurn) {
                    if (od_ON(cvs, distrib_n)) {
                        od->status_distrib = receive_od_distrib;
                        return -10;
                    }
                }
            }
            if (rpl->nTurns > od->rpl_nTurn) {
                od->status_distrib = S0_od_distrib;
                printf("od_distrib : abort\n");
                return 0;
            }
            return -10;
        }
        case receive_od_distrib: {
            if (pallet_n > -10) {
                od->status_distrib = S0_od_distrib;
                printf("pallet_n = %d\n", pallet_n);
                return pallet_n;
            }
            return -10;
        }
        default:
            return 0;
    }
}
