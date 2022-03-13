//
// Created by Louis Libert on 9/03/22.
//
#include <iostream>
#include <unistd.h>
#include <math.h>
#include <chrono>

#include "../ctrlStruct/ctrlStruct.h"


using namespace std::chrono;

int main()
{
    ctrlStruct *cvs;
    cvs = cvs_init();

    // variables declaration
    ctrlIn  *inputs;
    ctrlOut *outputs;
    lowLevelCtrl *llc;
    myPosition *mp;
    midLevelCtrlPF *mlcPF;
    double t;
    double dt;

    // variables initialization
    inputs  = cvs->inputs;
    outputs = cvs->outputs;
    llc  = cvs->llc;
    mp = cvs->mp;
    mlcPF = cvs->mlcPF;
    t = inputs->t;
    dt = inputs->dt;

    // loop
    double r_sp_ref = 0.0;
    double l_sp_ref = 0.0;

    while (inputs->t < 5) {
        auto start = high_resolution_clock::now();

        get_speeds_mes(cvs); // ctrlIn

        if (t >= 0 && t < 2) {
            r_sp_ref = 2.0;
            l_sp_ref = 2.0;
        }
        else if (t >= 2 && t < 4) {
            r_sp_ref = -4.0;
            l_sp_ref = -4.0;
        }
        else {
            r_sp_ref = 0.0;
            l_sp_ref = 0.0;
        }
        set_commands(cvs, r_sp_ref, l_sp_ref); // llc
        send_commands(cvs); // ctrlOut

        update_time(cvs);
        auto stop = high_resolution_clock::now();
        auto duration = duration_cast<microseconds>(stop - start);
        printf("duration.count() = %lld us | t = %f\n-------------\n", duration.count(), inputs->t);


        usleep(dt * 1000000 - duration.count());
    }

    return 0;
}