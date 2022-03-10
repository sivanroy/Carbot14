//
// Created by Louis Libert on 10/03/22.
//
#include <iostream>
#include <unistd.h>
#include <math.h>
#include <chrono>

#include "../ctrlStruct/ctrlStruct.h"


int main()
{
    ctrlStruct *cvs;
    cvs = cvs_init();

    // variables declaration
    CtrlIn  *inputs;
    CtrlOut *outputs;
    double t;
    double dt;

    // variables initialization
    inputs  = cvs->inputs;
    outputs = cvs->outputs;
    t = inputs->t;
    dt = inputs->dt;

    int r_cmd = 0;
    int l_cmd = 0;

    while (inputs->t < 5) {
        auto start = high_resolution_clock::now();

        get_speeds_mes(cvs); // ctrlIn

        printf("r_sp_mes_enc = %f | l_sp_mes_enc = %f\n", inputs->r_sp_mes_enc, inputs->l_sp_mes_enc);
        printf("r_sp_mes_odo = %f | l_sp_mes_odo = %f\n", inputs->r_sp_mes_odo, inputs->l_sp_mes_odo);

        if (t >= 0 && t < 2) {
            r_cmd = 10;
            l_cmd = 10;
        }
        else if (t >= 2 && t < 4) {
            r_cmd = 20;
            l_cmd = 20;
        }
        else if (t >= 4 && t < 6) {
            r_cmd = -10;
            l_cmd = 10;
        }
        else {
            r_cmd = 0;
            l_cmd = 0;
        }
        outputs->r_cmd = r_cmd;
        outputs->l_cmd = l_cmd
        send_commands(cvs); // ctrlOut

        update_time(cvs);
        auto stop = high_resolution_clock::now();
        auto duration = duration_cast<microseconds>(stop - start);
        printf("duration.count() = %lld us | t = %f\n-------------\n", duration.count(), inputs->t);

        usleep(dt * 1000000 - duration.count());
    }

    return 0;
}
