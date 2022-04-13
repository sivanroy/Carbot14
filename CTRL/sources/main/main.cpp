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

using namespace std;

int main()
{
    ctrlStruct *cvs;
    cvs = cvs_init();
    printf("cvs init : end\n");

    ctrlIn *inputs = cvs->inputs;
    ctrlOut *outputs = cvs->outputs;
    lowLevelCtrl *llc = cvs->llc;
    myPosition *mp = cvs->mp;
    midLevelCtrlPF *mlcPF = cvs->mlcPF;
    highLevelCtrlPF *hlcPF = cvs->hlcPF;
    rplStruct *rpl = cvs->rpl;
    pushShed *pshed = cvs->pshed;
    statAndShed *saShed = cvs->saShed;
    midLevelCtrl *mlc = cvs->mlc;
    oppPosition *op = cvs->op;
    mThreadsStruct *mt = cvs->mt;
    teensyStruct *teensy = cvs->teensy;
    reCalibStruct *rec = cvs->rec;
    double dt = inputs->dt;

    int display_position_ON = 0;
    int cmdON = 0;
    int llcON = 0;
    int mlcPF_ON = 0;
    int mlc_ON = 0;
    int rplON = 0;
    int odoCalib = 0;
    int hlcPFON = 0;
    int pushShedON = 0;
    int pushShed_and_sonar_ON = 0;
    int icp_test = 0;
    int icpON = 0;
    int teensyON = 0;
    int saShedON = 1;
    int distON = 0;

    int mThreadsON = 0;

    if (icpON) {
        cvs->mp->x = 3-0.14;//2.00;//3-0.14;
        cvs->mp->y = 1.13;//0.75;//0.445+0.125;//0.45;//2-0.53;
        cvs->mp->th = M_PI;//0;//M_PI;
        printf("x = %f | y = %f | th = %f\n",cvs->mp->x, cvs->mp->y, cvs->mp->th);

        IcpPointToPlane icp(rec->map_p,rec->M,2);
        icp.setMaxIterations(rec->max_iter);
        icp.setMinDeltaParam(rec->min_delta);

        int i;
        for (i = 0; i < rec->M*2; i+=2) {
            fprintf(cvs->icp1_data, "%f,%f\n", rec->map_p[i], rec->map_p[i+1]);
        }
        printf("M = %d\n", rec->M);
        while (rpl->nTurns < 3) {
            rpl_grabData(cvs);
            rec_ICP(cvs, &icp);
            break;
        }
    }
    if (icp_test) {
        // define a 3 dim problem with 10000 model points
        // and 10000 template points:
        int32_t dim = 2;
        int32_t numM = 1000;
        int32_t numT = 200;

        // allocate model and template memory
        double* M = (double*)calloc(2*numM,sizeof(double));
        double* T = (double*)calloc(2*numT,sizeof(double));
        double* S = (double*)calloc(2*numT,sizeof(double));

        // set model and template points
        cout << endl << "Creating model with 10000 points ..." << endl;
        cout << "Creating template by shifting model by (1,0.5,-1) ..." << endl;
        int32_t k=0;
        double x;
        double xa;
        double ya;
        double M_end = (double) 2/numM;
        double T_end = (double) 2/numT;
        printf("M_end = %f | T_end = %f\n", M_end, T_end);
        for (x=0; x<2; x+=M_end) {
            xa = x;
            ya = x;
            M[k * dim + 0] = xa;
            M[k * dim + 1] = ya;
            fprintf(cvs->icp1_data, "%f,%f\n", xa, ya);
            k++;
        }
        k = 0;
        for (x=0; x<2; x+=T_end) {
            xa = x+0.013;// work until 0.014
            ya = x+0.013;// + 0.2*(x*0.05); // work until 0.014
            T[k*dim+0] = xa;
            T[k*dim+1] = ya;
            fprintf(cvs->icp2_data, "%f,%f\n", xa, ya);
            k++;
        }

        printf("for for\n");
        // start with identity as initial transformation
        // in practice you might want to use some kind of prediction here
        Matrix R = Matrix::eye(dim);
        Matrix t(dim,1);

        // run point-to-plane ICP (-1 = no outlier threshold)
        cout << endl << "Running ICP (point-to-plane, no outliers)" << endl;
        IcpPointToPlane icp(M,numM,dim);
        icp.fit(T,numT,R,t,-1);

        int i;
        double Sx;
        double Sy;
        for (i = 0; i < numT*2; i+=2) {
            Sx = R.val[0][0] * T[i] + R.val[0][1] * T[i+1] + t.val[0][0];
            Sy = R.val[1][0] * T[i] + R.val[1][1] * T[i+1] + t.val[1][0];
            fprintf(cvs->icp3_data, "%f,%f\n", Sx, Sy);
            S[i] = Sx;
            S[i+1] = Sy;
        }

        // results
        cout << endl << "Transformation results:" << endl;
        cout << "R:" << endl << R << endl << endl;
        cout << "t:" << endl << t << endl << endl;
        //cout << "Residual:"<<residual;

        // free memory
        printf("free1\n");
        //free(M);
        //free(T);
        printf("free2\n");
    }
    if (mThreadsON) {
        threads_start(cvs);

        pushShed_launch(cvs);
        cvs->mp->x = 3-0.14;//3-0.94;//3-0.14;
        cvs->mp->y = 2-0.53;//0.795+0.125;//1.13;//0.45;//2-0.53;
        cvs->mp->th = M_PI;//0;//M_PI;

        double i_time = 20;
        double c_time = 1;
        double t_end = 15;
        while (inputs->t < t_end) {

            auto start = high_resolution_clock::now();

            //printf("-------------\main loop\n");
            esquive_loop(cvs);
            //dyn_obs_set(cvs);

            if (inputs->t > i_time*c_time) {
                rec_ON(cvs);
                c_time++;
            }
            update_time(cvs);
            auto stop = high_resolution_clock::now();
            auto duration = duration_cast<microseconds>(stop - start);
            printf("duration.count() = %lld us | t = %f\n-------------\n", duration.count(), inputs->t);
            fprintf(cvs->llc_data, "%f,%f,%f,%f,%f,%f,%f\n", inputs->t, mlcPF->r_sp_ref, mlcPF->l_sp_ref, inputs->r_sp_mes_enc, inputs->l_sp_mes_enc, inputs->r_sp_mes_odo, inputs->l_sp_mes_odo);
            usleep(dt * 1000000 - duration.count());
        }


        mt->thread_main_end = 1;
        printf("th_end : start\n");
        threads_end(cvs);
        printf("th_end : end\n");
    }
    if (display_position_ON){
        cvs->mp->x = 3-0.14-0.0625;
        cvs->mp->y = 1.13;
        cvs->mp->th = M_PI;

        mlcPF->t_start = inputs->t;
        while (inputs->t < 50) {
            auto start = high_resolution_clock::now();

            //if (inputs->t >= 2 && inputs->t < 4) th_ref = 0;//-M_PI/4;

            get_d2r_data(cvs); // ctrlIn

            set_new_position(cvs);
            printf("x = %f | y = %f | th = %f\n", mp->x, mp->y, mp->th);

            update_time(cvs);
            auto stop = high_resolution_clock::now();
            auto duration = duration_cast<microseconds>(stop - start);
            usleep(dt * 1000000 - duration.count());
        }
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

        while (inputs->t < 2) {
            auto start = high_resolution_clock::now();

            get_d2r_data(cvs); // ctrlIn

            printf("r_sp_mes_enc = %f | l_sp_mes_enc = %f\n", inputs->r_sp_mes_enc, inputs->l_sp_mes_enc);
            printf("r_sp_mes_odo = %f | l_sp_mes_odo = %f\n", inputs->r_sp_mes_odo, inputs->l_sp_mes_odo);

            if (inputs->t >= 0 && inputs->t < 5) {
                r_sp_ref = -10;
                l_sp_ref = -10;
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

        mp->th = -M_PI;
        double v_ref = -0.1;
        double th_ref = -M_PI;

        mlcPF->t_start = inputs->t;
        while (inputs->t < mlcPF->t_start + 5) {

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
            int d_sFR = d2r_enc_measure(cvs,-1,0,1, true);
            int d_sFL = d2r_enc_measure(cvs,-1,1,1, true);
            int d_sBR = d2r_enc_measure(cvs,-1,0,0, true);
            int d_sBL = d2r_enc_measure(cvs,-1,1,0, true);

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

        double x_goal = 3-0.57;
        double y_goal = 1.13;

        cvs->mp->x = 3-0.27;
        cvs->mp->y = 1.13;
        cvs->mp->th = 0;

        while (inputs->t < 10) {
            auto start = high_resolution_clock::now();

            get_d2r_data(cvs); // ctrlIn

            printf("r_sp_mes_enc = %f | l_sp_mes_enc = %f\n", inputs->r_sp_mes_enc, inputs->l_sp_mes_enc);
            printf("r_sp_mes_odo = %f | l_sp_mes_odo = %f\n", inputs->r_sp_mes_odo, inputs->l_sp_mes_odo);

            set_speed_ref(cvs,x_goal,y_goal,0);
            if(mlc->reach_goal){
                printf("reached goal\n");
                break;
            }
            set_commands(cvs, mlc->r_sp_ref, mlc->l_sp_ref);
            printf("cmd_r = %d | cmd_l = %d\n", outputs->r_cmd, outputs->l_cmd);
            send_commands(cvs);

            set_new_position(cvs);
            //printf("x = %f | y = %f | th = %f\n", mp->x, mp->y, mp->th);

            fprintf(cvs->llc_data, "%f,%f,%f,%f,%f,%f,%f\n", inputs->t, mlcPF->r_sp_ref, mlcPF->l_sp_ref, inputs->r_sp_mes_enc, inputs->l_sp_mes_enc, inputs->r_sp_mes_odo, inputs->l_sp_mes_odo);

            //teensy_recv(cvs);

            update_time(cvs);
            auto stop = high_resolution_clock::now();
            auto duration = duration_cast<microseconds>(stop - start);
            //printf("duration.count() = %lld us | t = %f\n-------------\n", duration.count(), inputs->t);

            usleep(dt * 1000000 - duration.count());
        }
    }
    if (hlcPFON) {
        double xgoal = cvs->mp->x;double ygoal=cvs->mp->y;
        int forward;double orientation;
        set_param_normal(cvs);
        cvs->mp->x = 3-0.27;
        cvs->mp->y = 1.13;
        cvs->mp->th = 0;
        //threads_start(cvs);


        printf("begin test hlcPF\n");
        while (inputs->t < 15) {
            auto start = high_resolution_clock::now();
            double t = inputs->t;
            if (t==0) {
                set_param_normal(cvs);
                xgoal = 2;//2.2;//1.2;
                ygoal = 1.5;//1.60;
                forward=-1;
                orientation = M_PI/2;
                set_goal(cvs, xgoal, ygoal, orientation);
                printf("goal A\n");
            } else if (t>10 & t<10.01) {
                set_param_prec(cvs);
                xgoal = 2;
                ygoal = .75;
                forward =0;
                orientation = M_PI/2;
                set_goal(cvs, xgoal, ygoal, orientation);
                printf("goal B\n");
            } else if (t>20 & t<20.1) {
                xgoal = 1;//1.3;
                ygoal = .5;
                forward =0;
                orientation = -M_PI;
                set_goal(cvs, xgoal, ygoal, orientation);
                printf("goal c\n");
            }

            get_d2r_data(cvs); // ctrlIn
            //printf("r_sp_mes_enc = %f | l_sp_mes_enc = %f\n", inputs->r_sp_mes_enc, inputs->l_sp_mes_enc);
            //printf("r_sp_mes_odo = %f | l_sp_mes_odo = %f\n", inputs->r_sp_mes_odo, inputs->l_sp_mes_odo);
            hlcPF_out(cvs, forward,1);
            //if(hlcPF->output) {
                //hlcPF->v_ref = 0;
                //hlcPF->theta_ref = 0;
                //break;
            //}
            mlcPF_out(cvs, hlcPF->v_ref, hlcPF->theta_ref);

            fprintf(cvs->tau_data, "%f,%f,%f\n", inputs->t, hlcPF->v_ref, tau_compute(cvs));

            //printf("v_ref; d %f \n", hlcPF->v_ref,hlcPF->d);
            //printf("hlcPF->v %f | hlcPF->theta %f | d %f \n",hlcPF->v_ref,hlcPF->theta_ref,hlcPF->d );
            set_commands(cvs, mlcPF->r_sp_ref, mlcPF->l_sp_ref);
            send_commands(cvs);

            set_new_position(cvs);
            //printf("cmd_r = %d | cmd_l = %d\n", outputs->r_cmd, outputs->l_cmd);
            //printf("x = %f | y = %f | th = %f\n", mp->x, mp->y, mp->th);

            fprintf(cvs->llc_data, "%f,%f,%f,%f,%f,%f,%f\n", inputs->t, mlcPF->r_sp_ref, mlcPF->l_sp_ref, inputs->r_sp_mes_enc, inputs->l_sp_mes_enc, inputs->r_sp_mes_odo, inputs->l_sp_mes_odo);

            update_time(cvs);
            auto stop = high_resolution_clock::now();
            auto duration = duration_cast<microseconds>(stop - start);

            //printf("duration.count() = %lld us | t = %f\n-------------\n", duration.count(), inputs->t);
            usleep(dt * 1000000 - duration.count());


        }
        printf("x = %f | y = %f | th = %f\n", mp->x, mp->y, mp->th);
        //mt->thread_main_end = 1;

        //threads_end(cvs);
    }
    if (pushShedON){
        pushShed_launch(cvs);
        printf("pushShedON\n");
        cvs->mp->x = 3-0.14-0.0625;
        cvs->mp->y = 2-0.53;
        cvs->mp->th = M_PI;

        while(inputs->t < 20){
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
    if (pushShed_and_sonar_ON){
        pushShed_launch(cvs);
        printf("pushShedON\n");
        cvs->mp->x = 3-0.14;
        cvs->mp->y = 2-0.53;
        cvs->mp->th = M_PI;

        while(inputs->t < 70){
            auto start = high_resolution_clock::now();
            if(inputs->l_front_s < 5 | inputs->r_front_s<5){
                motors_stop(cvs);
            }
            else {
                pushShed_loop(cvs);
            }
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
    if (saShedON){
        saShed_launch(cvs);
        printf("sasShedON\n");
        cvs->mp->x = 3-0.14;
        cvs->mp->y = 2-0.53;
        cvs->mp->th = M_PI;

        while(inputs->t < 10){
            auto start = high_resolution_clock::now();

            saShed_loop(cvs);
            if(saShed->output) {
                printf("ended with %d\n",saShed->output);
                motors_stop(cvs);
                break;
            }
            update_time(cvs);
            auto stop = high_resolution_clock::now();
            auto duration = duration_cast<microseconds>(stop - start);
            usleep(dt * 1000000 - duration.count());
        }
    }
    if (distON) {
        threads_start(cvs);

        distr_launch(cvs);
        printf("distON\n");
        cvs->mp->x = 3-0.14;
        cvs->mp->y = 2-0.53;
        cvs->mp->th = M_PI;

        while(inputs->t < 20){
            auto start = high_resolution_clock::now();

            distr_loop(cvs);
            if(cvs->distr->output) {
                printf("ended with %d\n",cvs->distr->output);
                motors_stop(cvs);
                break;
            }
            fprintf(cvs->llc_data, "%f,%f,%f,%f,%f,%f,%f\n", inputs->t, mlcPF->r_sp_ref, mlcPF->l_sp_ref, inputs->r_sp_mes_enc, inputs->l_sp_mes_enc, inputs->r_sp_mes_odo, inputs->l_sp_mes_odo);
            fprintf(cvs->tau_data, "%f,%f,%f\n", inputs->t, hlcPF->v_ref, tau_compute(cvs));

            update_time(cvs);
            auto stop = high_resolution_clock::now();
            auto duration = duration_cast<microseconds>(stop - start);
            usleep(dt * 1000000 - duration.count());
        }
        usleep(3000000);
        mt->thread_main_end = 1;
        printf("th_end : start\n");
        threads_end(cvs);
        printf("th_end : end\n");

    }
    if (teensyON) {
        int A = 0;
        int B = 0;
        int C = 0;
        int D = 0;
        int K = 0;
        int Q = 0;
        int R = 0;
        int S = 0;
        while (inputs->t < 13) {

            auto start = high_resolution_clock::now();

            teensy_recv(cvs);

            //printf("switch_F : %d\n", teensy->switch_F);
            if (teensy->switch_F && teensy->switch_F_end == 0) {
                teensy_send(cvs, "5");
                teensy->switch_F = 0;
                teensy->switch_F_end = 1;
            }
            if (teensy->switch_B && teensy->switch_B_end == 0) {
                //teensy_send(cvs, "5");
                printf("Switch back ON\n");
                teensy->switch_B = 0;
                teensy->switch_B_end = 1;
            }

            if (inputs->t >= 1 && B == 0) {
                teensy_send(cvs, "A");
                B = 1;
            }

            if (inputs->t >= 3 && C == 0) {
                teensy_send(cvs, "C");
                C = 1;
            }

            if (inputs->t >= 5 && D == 0) {
                teensy_send(cvs, "D");
                D = 1;
            }
            if (inputs->t >= 7 && R == 0) {
                teensy_send(cvs, "R");
                R = 1;
            }
            if (inputs->t >= 9 && S == 0) {
                teensy_send(cvs, "S");
                S = 1;
            }
            if (inputs->t >= 11 && Q == 0) {
                teensy_send(cvs, "Q");
                Q = 1;
            }


            update_time(cvs);
            auto stop = high_resolution_clock::now();
            auto duration = duration_cast<microseconds>(stop - start);
            //printf("duration.count() = %lld us | t = %f\n-------------\n", duration.count(), inputs->t);

            usleep(dt * 1000000 - duration.count());
        }
    }
    usleep(2000000);
    motors_stop(cvs);
    teensy_send(cvs, "B");
    usleep(2000000);
    teensy_send(cvs, "R");
    cvs_free(cvs);

    return 0;
}

