/*!
 * \file oppPosition.cpp
 * \brief Detection of the opponent position
 */

#include "oppPosition.h"


void op_init(oppPosition *op)
{
    op->n_opp = 0;
    op->cluster_size_min = 4;
    op->map_margin = 0.05;

    op->update_flag = 0;
    op->x_op = -1;
    op->y_op = -1;
}

void get_opp_pos(ctrlStruct *cvs)
{
    printf("opp_pos\n");
    myPosition *mp;
    oppPosition *op;
    rplStruct *rpl;
    mThreadsStruct *mt;

    mp = cvs->mp;
    op = cvs->op;
    rpl = cvs->rpl;
    mt = cvs->mt;

    int size = rpl->data_size;

    double map_margin = op->map_margin;

    double x = mp->x;
    double y = mp->y;
    double th = mp->th;

    double a;
    double d;
    double pt_or;
    double pt_x;
    double pt_y;

    int size_op = 0;
    double x_op[8192];
    double y_op[8192];
    double d_op[8192];

    int i;
    for (i = 0; i < size; i++) {
        a = rpl->a[i];
        d = rpl->d[i]/1000;
        pt_or = th - a;
        pt_x = x + d * cos(pt_or);
        pt_y = y + d * sin(pt_or);

        if ((pt_x > 0 + map_margin && pt_x < 3 - map_margin) && (pt_y > 0 + map_margin && pt_y < 2 - map_margin)) {
            x_op[size_op] = pt_x;
            y_op[size_op] = pt_y;
            printf("pt_x = %f | pt_y = %f\n", pt_x, pt_y);
            d_op[size_op] = d;
            size_op++;
        }
    }
    if (size_op == 0) op->n_opp = 0;
    else {
        double r_cluster = 0.20;
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
        if (max_cluster > 0) { //op->cluster_size_min
            pthread_mutex_lock(&(mt->mutex_op));
            op->x_op = x_op[n_cluster];
            op->y_op = y_op[n_cluster];
            printf("opp : x_op = %f | y_op = %f\n", op->x_op, op->y_op);
            op->update_flag = 1;
            pthread_mutex_unlock(&(mt->mutex_op));
        }
        else {
            pthread_mutex_lock(&(mt->mutex_op));
            op->x_op = -1;
            op->y_op = -1;
            op->update_flag = 1;
            printf("opp : x_op = %f | y_op = %f\n", op->x_op, op->y_op);
            pthread_mutex_unlock(&(mt->mutex_op));
        }
    }
}


