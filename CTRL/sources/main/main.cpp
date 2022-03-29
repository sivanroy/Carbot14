//
// Created by Louis Libert on 10/03/22.
//
#include <iostream>
#include <unistd.h>
#include <math.h>
#include <chrono>
#include <stdlib.h>

#include "../ctrlStruct/ctrlStruct.h"

using namespace std::chrono;


int main()
{
    ctrlStruct *cvs;
    cvs = cvs_init();
    printf("dt = %f\n", cvs->inputs->dt);
    printf("cvs init : end\n");

    // variables declaration
    ctrlIn  *inputs;
    ctrlOut *outputs;
    lowLevelCtrl *llc;
    myPosition *mp;
    midLevelCtrlPF *mlcPF;
    rplStruct *rpl;
    highLevelCtrlPF *hlcPF;
    pushShed *pshed;
    midLevelCtrl *mlc;
    oppPosition *op;
    mThreadsStruct *mt;
    teensyStruct *teensy;
    double dt;

    // variables initialization
    inputs  = cvs->inputs;
    outputs = cvs->outputs;
    llc  = cvs->llc;
    mp = cvs->mp;
    mlcPF = cvs->mlcPF;
    hlcPF = cvs->hlcPF;
    rpl = cvs->rpl;
    pshed = cvs->pshed;
    mlc = cvs->mlc;
    op = cvs->op;
    mt = cvs->mt;
    teensy = cvs->teensy;
    dt = inputs->dt;

    int cmdON = 0;
    int llcON = 1;
    int mlcPF_ON = 0;
    int mlc_ON = 0;
    int rplON = 0;
    int odoCalib = 0;
    int hlcPFON = 0;
    int pushShedON = 0;
    int teensyON = 0;
    int mThreadsON = 1;

    if (mThreadsON) {
        threads_launcher(cvs);
    }

    if (cmdON) {
        int r_cmd = 0;
        int l_cmd = 0;

        while (inputs->t < 2) {
            auto start = high_resolution_clock::now();

            get_d2r_data(cvs); // ctrlIn

            printf("r_sp_mes_enc = %f | l_sp_mes_enc = %f\n", inputs->r_sp_mes_enc, inputs->l_sp_mes_enc);
            printf("r_sp_mes_odo = %f | l_sp_mes_odo = %f\n", inputs->r_sp_mes_odo, inputs->l_sp_mes_odo);

            if (inputs->t >= 0 && inputs->t < 1) {
                r_cmd = 5;
                l_cmd = 5;
            }

            else if (inputs->t >= 1 && inputs->t < 2) {
                r_cmd = 10;
                l_cmd = 10;
            }
                /*
                else if (inputs->t >= 4 && inputs->t < 6) {
                    r_cmd = -10;
                    l_cmd = 10;
                }
                */
            else {
                r_cmd = 0;
                l_cmd = 0;
            }
            outputs->r_cmd = r_cmd;
            outputs->l_cmd = l_cmd;
            send_commands(cvs); // ctrlOut

            fprintf(cvs->llc_data, "%f,%d,%d,%f,%f\n", inputs->t, r_cmd, l_cmd, inputs->r_sp_mes_enc, inputs->l_sp_mes_enc);

            update_time(cvs);
            auto stop = high_resolution_clock::now();
            auto duration = duration_cast<microseconds>(stop - start);
            printf("duration.count() = %lld us | t = %f\n-------------\n", duration.count(), inputs->t);

            usleep(dt * 1000000 - duration.count());
        }
    }
    if (llcON) {
        double r_sp_ref = 0.0;
        double l_sp_ref = 0.0;

        while (inputs->t < 4) {
            auto start = high_resolution_clock::now();

            get_d2r_data(cvs); // ctrlIn

            printf("r_sp_mes_enc = %f | l_sp_mes_enc = %f\n", inputs->r_sp_mes_enc, inputs->l_sp_mes_enc);
            printf("r_sp_mes_odo = %f | l_sp_mes_odo = %f\n", inputs->r_sp_mes_odo, inputs->l_sp_mes_odo);

            if (inputs->t >= 0 && inputs->t < 3) {
                r_sp_ref = 10;
                l_sp_ref = 10;
            }

            else if (inputs->t >= 1.5 && inputs->t < 3) {
                r_sp_ref = 10;
                l_sp_ref = 10;
            }
                /*
                else if (inputs->t >= 4 && inputs->t < 6) {
                    r_cmd = -10;
                    l_cmd = 10;
                }
                */
            else {
                r_sp_ref = 0;
                l_sp_ref = 0;
            }
            set_commands(cvs, r_sp_ref, l_sp_ref);
            printf("cmd_r = %d | cmd_l = %d\n", outputs->r_cmd, outputs->l_cmd);
            send_commands(cvs);

            set_new_position(cvs);
            printf("x = %f | y = %f | th = %f\n", mp->x, mp->y, mp->th);

            fprintf(cvs->llc_data, "%f,%f,%f,%f,%f,%f,%f\n", inputs->t, r_sp_ref, l_sp_ref, inputs->r_sp_mes_enc, inputs->l_sp_mes_enc, inputs->r_sp_mes_odo, inputs->l_sp_mes_odo);

            update_time(cvs);
            auto stop = high_resolution_clock::now();
            auto duration = duration_cast<microseconds>(stop - start);
            printf("duration.count() = %lld us | t = %f\n-------------\n", duration.count(), inputs->t);

            usleep(dt * 1000000 - duration.count());
        }
    }
    if (mlcPF_ON) {
        mp->th = -M_PI/4;
        double v_ref = -0.1;
        double th_ref = -M_PI/4;

        mlcPF->t_start = inputs->t;
        while (inputs->t < mlcPF->t_start + 2) {
            auto start = high_resolution_clock::now();

            //if (inputs->t >= 2 && inputs->t < 4) th_ref = 0;//-M_PI/4;

            get_d2r_data(cvs); // ctrlIn

            printf("r_sp_mes_enc = %f | l_sp_mes_enc = %f\n", inputs->r_sp_mes_enc, inputs->l_sp_mes_enc);
            printf("r_sp_mes_odo = %f | l_sp_mes_odo = %f\n", inputs->r_sp_mes_odo, inputs->l_sp_mes_odo);

            mlcPF_out(cvs, v_ref, th_ref);
            set_commands(cvs, mlcPF->r_sp_ref, mlcPF->l_sp_ref);
            printf("cmd_r = %d | cmd_l = %d\n", outputs->r_cmd, outputs->l_cmd);
            send_commands(cvs);

            set_new_position(cvs);
            printf("x = %f | y = %f | th = %f\n", mp->x, mp->y, mp->th);

            fprintf(cvs->llc_data, "%f,%f,%f,%f,%f,%f,%f\n", inputs->t, mlcPF->r_sp_ref, mlcPF->l_sp_ref, inputs->r_sp_mes_enc, inputs->l_sp_mes_enc, inputs->r_sp_mes_odo, inputs->l_sp_mes_odo);

            update_time(cvs);
            auto stop = high_resolution_clock::now();
            auto duration = duration_cast<microseconds>(stop - start);
            printf("duration.count() = %lld us | t = %f\n-------------\n", duration.count(), inputs->t);

            usleep(dt * 1000000 - duration.count());
        }
    }
    if (rplON) {
        while (rpl->nTurns < 20) {
            auto start = high_resolution_clock::now();

            int grabSuccess = rpl_grabData(cvs);
            printf("grabSuccess = %d\n", grabSuccess);
            /*
            for (int i = 0; i < rpl->data_size; i++) {
                printf("a = %f | d = %f | q = %f\n", rpl->a[i], rpl->d[i], rpl->q[i]);
            }
            */
            auto stop = high_resolution_clock::now();
            auto duration = duration_cast<microseconds>(stop - start);
            printf("\nduration.count() = %lld us\n-------------\n", duration.count());

            //usleep(dt * 1000000 - duration.count());
        }
    }
    if (odoCalib) {
        int r_ticks_enc_tot = 0;
        int l_ticks_enc_tot = 0;
        int r_ticks_odo_tot = 0;
        int l_ticks_odo_tot = 0;

        while (inputs->t < 10) {
            auto start = high_resolution_clock::now();

            int r_ticks_enc = d2r_enc_measure(cvs, 1, 0, -1 ,true);
            int l_ticks_enc = d2r_enc_measure(cvs, 1, 1, -1 ,true);
            int r_ticks_odo = d2r_enc_measure(cvs, 0, 0, -1 ,true);
            int l_ticks_odo = d2r_enc_measure(cvs, 0, 1, -1 ,true);
            int d1 = d2r_enc_measure(cvs,-1,-1,0,true);
            int d2 = d2r_enc_measure(cvs,-1,-1,1,true);

            r_ticks_enc_tot += r_ticks_enc;
            l_ticks_enc_tot += l_ticks_enc;
            r_ticks_odo_tot += r_ticks_odo;
            l_ticks_odo_tot += l_ticks_odo;

            update_time(cvs);
            auto stop = high_resolution_clock::now();
            auto duration = duration_cast<microseconds>(stop - start);
            printf("duration.count() = %lld us | t = %f\n-------------\n", duration.count(), inputs->t);

            usleep(dt * 1000000 - duration.count());
        }
        printf("r_ticks_enc_tot = %d | l_ticks_enc_tot = %d\n", r_ticks_enc_tot, l_ticks_enc_tot);
        printf("r_ticks_odo_tot = %d | l_ticks_odo_tot = %d\n", r_ticks_odo_tot, l_ticks_odo_tot);
    }

    if (mlc_ON){
        double x_goal = 2.6;
        double y_goal = 0.4;

        mp->x = 2.4;
        mp->y = 0.6;
        mp->th = -1.3*M_PI/4;

        while (inputs->t < 3) {
            auto start = high_resolution_clock::now();

            get_d2r_data(cvs); // ctrlIn

            printf("r_sp_mes_enc = %f | l_sp_mes_enc = %f\n", inputs->r_sp_mes_enc, inputs->l_sp_mes_enc);
            printf("r_sp_mes_odo = %f | l_sp_mes_odo = %f\n", inputs->r_sp_mes_odo, inputs->l_sp_mes_odo);

            //mlcPF_out(cvs, v_ref, th_ref);
            set_speed_ref(cvs,x_goal,y_goal);
            if(mlc->reach_goal){
                break;
            }
            set_commands(cvs, mlc->r_sp_ref, mlc->l_sp_ref);
            printf("cmd_r = %d | cmd_l = %d\n", outputs->r_cmd, outputs->l_cmd);
            send_commands(cvs);

            set_new_position(cvs);
            printf("x = %f | y = %f | th = %f\n", mp->x, mp->y, mp->th);

            fprintf(cvs->llc_data, "%f,%f,%f,%f,%f,%f,%f\n", inputs->t, mlcPF->r_sp_ref, mlcPF->l_sp_ref, inputs->r_sp_mes_enc, inputs->l_sp_mes_enc, inputs->r_sp_mes_odo, inputs->l_sp_mes_odo);

            teensy_recv(cvs);

            update_time(cvs);
            auto stop = high_resolution_clock::now();
            auto duration = duration_cast<microseconds>(stop - start);
            printf("duration.count() = %lld us | t = %f\n-------------\n", duration.count(), inputs->t);

            usleep(dt * 1000000 - duration.count());
        }
    }

    if (hlcPFON) {
        double x_goal = 2;
        double y_goal = 1;

        cvs->mp->x = 3-0.14;
        cvs->mp->y = 1.13;
        cvs->mp->th = M_PI;

        printf("begin test hlcPF\n");
        while (inputs->t < 10) {
            auto start = high_resolution_clock::now();

            get_d2r_data(cvs); // ctrlIn

            printf("r_sp_mes_enc = %f | l_sp_mes_enc = %f\n", inputs->r_sp_mes_enc, inputs->l_sp_mes_enc);
            printf("r_sp_mes_odo = %f | l_sp_mes_odo = %f\n", inputs->r_sp_mes_odo, inputs->l_sp_mes_odo);


            main_pot_force(cvs,x_goal,y_goal);

            if(hlcPF->output) {
                break;
            }

            mlcPF_out(cvs, hlcPF->v_ref, hlcPF->theta_ref);
            printf("hlcPF->v %f | hlcPF->theta %f\n",hlcPF->v_ref,hlcPF->theta_ref );
            set_commands(cvs, mlcPF->r_sp_ref, mlcPF->l_sp_ref);
            send_commands(cvs);

            set_new_position(cvs);
            printf("cmd_r = %d | cmd_l = %d\n", outputs->r_cmd, outputs->l_cmd);
            printf("x = %f | y = %f | th = %f\n", mp->x, mp->y, mp->th);

            fprintf(cvs->llc_data, "%f,%f,%f,%f,%f,%f,%f\n", inputs->t, mlcPF->r_sp_ref, mlcPF->l_sp_ref, inputs->r_sp_mes_enc, inputs->l_sp_mes_enc, inputs->r_sp_mes_odo, inputs->l_sp_mes_odo);

            update_time(cvs);
            auto stop = high_resolution_clock::now();
            auto duration = duration_cast<microseconds>(stop - start);

            printf("duration.count() = %lld us | t = %f\n-------------\n", duration.count(), inputs->t);
            usleep(dt * 1000000 - duration.count());


        }
    }

    if (pushShedON){
        pushShed_launch(cvs);
        printf("pushShedON\n");
        cvs->mp->x = 3-0.14;
        cvs->mp->y = 2-0.53;
        cvs->mp->th = M_PI;

        while(inputs->t < 70){
            auto start = high_resolution_clock::now();

            pushShed_loop(cvs);
            if(pshed->output) {
                printf("ended\n");
                motors_stop(cvs);
                break;
            }
            update_time(cvs);
            auto stop = high_resolution_clock::now();
            auto duration = duration_cast<microseconds>(stop - start);
            usleep(dt * 1000000 - duration.count());
        }
    }

    if (teensyON) {
        int A = 0;
        int B = 0;
        while (inputs->t < 8) {
            auto start = high_resolution_clock::now();

            teensy_recv(cvs);

            //printf("switch_F : %d\n", teensy->switch_F);
            if (teensy->switch_F && teensy->switch_F_end == 0) {
                teensy_send(cvs, "5");
                teensy->switch_F = 0;
                teensy->switch_F_end = 1;
            }

            if (inputs->t >= 1 && A == 0) {
                teensy_send(cvs, "A");
                A = 1;
            }
            if (inputs->t >= 12 && B == 0) {
                teensy_send(cvs, "B");
                B = 1;
            }


            update_time(cvs);
            auto stop = high_resolution_clock::now();
            auto duration = duration_cast<microseconds>(stop - start);
            //printf("duration.count() = %lld us | t = %f\n-------------\n", duration.count(), inputs->t);

            usleep(dt * 1000000 - duration.count());
        }
    }
    motors_stop(cvs);
    teensy_send(cvs, "B");
    usleep(2000000);
    teensy_send(cvs, "R");
    cvs_free(cvs);

    return 0;
}

