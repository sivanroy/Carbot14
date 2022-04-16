/*!
 * \file oppPosition.cpp
 * \brief Detection of the opponent position
 */

#include "oppPosition.h"


void op_init(oppPosition *op)
{
    op->n_opp = 0;
    op->cluster_size_min = 4;
    op->cluster_r = 0.05;
    op->map_margin = 0.15;

    op->no_opp = 0;

    op->update_flag = 0;
    op->w_limit = 0.5;
    op->x_op = -1;
    op->y_op = -1;
}

int not_wall(ctrlStruct *cvs, double x, double y)
{
    oppPosition *op = cvs->op;
    double map_margin = op->map_margin;

    if ((x > 0 + map_margin && x < 3 - map_margin) && (y > 0 + map_margin && y < 2 - map_margin)) {

        if ((sqrt((x-0)*(x-0) + (y-0)*(y-0)) > (0.51 - map_margin + 0.06)) &&
                (sqrt((x-3)*(x-3) + (y-0)*(y-0)) > (0.51 - map_margin + 0.06))) {
            return 1;
        }
    }
    return 0;
}

void get_opp_pos(ctrlStruct *cvs)
{
    myPosition *mp = cvs->mp;
    oppPosition *op = cvs->op;
    rplStruct *rpl = cvs->rpl;
    mThreadsStruct *mt = cvs->mt;

    int size = rpl->data_size;
    double map_margin = op->map_margin;

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

    int size_op = 0;
    double x_op[8192];
    double y_op[8192];

    int i;
    for (i = 0; i < size; i++) {
        a = rpl->a[i];
        d = rpl->d[i];
        pt_or = th - a;
        pt_x = (x + e * cos(th)) + (d * cos(pt_or));//
        pt_y = (y + e * sin(th)) + (d * sin(pt_or));//
        fprintf(cvs->rpl_data, "%f,%f\n", pt_x, pt_y);
        if (not_wall(cvs, pt_x, pt_y)) {
            x_op[size_op] = pt_x;
            y_op[size_op] = pt_y;
            size_op++;
            //fprintf(cvs->op_data, "%f,%f\n", pt_x, pt_y);
        }
    }
    if (size_op == 0 || op->no_opp) {
        pthread_mutex_lock(&(mt->mutex_op));
        op->x_op = -1;
        op->y_op = -1;
        op->update_flag = 1;
        pthread_mutex_unlock(&(mt->mutex_op));
    }
    else {
        double r_cluster = op->cluster_r;
        int counter_cluster[size_op];
        for (i = 0; i < size_op; i++) counter_cluster[i] = 0;

        int j;
        double dist;
        for (i = 0; i < size_op; i++) {
            for (j = 0; j < size_op; j++) {
                dist = sqrt((x_op[i] - x_op[j])*(x_op[i] - x_op[j]) + (y_op[i] - y_op[j])*(y_op[i] - y_op[j]));
                if (dist < r_cluster) counter_cluster[i] = counter_cluster[i] + 1;
            }
        }
        int max_cluster = 0;
        int n_cluster = 0;
        int n;

        double dist_cluster = 9999;
        for (i = 0; i < size_op; i++) {
            n = counter_cluster[i];
            dist = sqrt((x_op[i] - x)*(x_op[i] - x) + (y_op[i] - y)*(y_op[i] - y));

            if (n > max_cluster) {
                max_cluster = n;
                dist_cluster = dist;
                n_cluster = i;
            }
            else if (n == max_cluster && dist < dist_cluster) {
                dist_cluster = dist;
                n_cluster = i;
            }
        }
        if (max_cluster > op->cluster_size_min) {
            pthread_mutex_lock(&(mt->mutex_op));
            op->x_op = x_op[n_cluster];
            op->y_op = y_op[n_cluster];
            op->update_flag = 1;
            //Sfprintf(cvs->op_data, "%f,%f\n", x_op[n_cluster], y_op[n_cluster]);
            printf("opp : x_op = %f | y_op = %f\n", op->x_op, op->y_op);
            pthread_mutex_unlock(&(mt->mutex_op));
        }
        else {
            pthread_mutex_lock(&(mt->mutex_op));
            op->x_op = -1;
            op->y_op = -1;
            op->update_flag = 1;
            printf("opp -1 -1 : x_op = %f | y_op = %f\n", op->x_op, op->y_op);
            pthread_mutex_unlock(&(mt->mutex_op));
        }
    }
}


