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
    objDetection *od = cvs->od;
    poseStatuette *poseStat = cvs->poseStat;
    excSquares *excSq = cvs->excSq;
    double dt = inputs->dt;

    int display_position_ON = 0;
    int cmdON = 0;
    int llcON = 0;
    int mlcPF_ON = 0;
    int mlc_ON = 0;
    int rplON = 0;
    int odoCalib = 0;   
    int hlcPFON = 0;
    int checkRepetabilityHLCPFTrianlge =0;
    int odo_caracterisation_ON = 0;
    int pushShedON = 0;
    int pushShed_and_sonar_ON = 0;
    int icp_test = 0;
    int icpON = 0;
    int teensyON = 0;
    int icpDynON = 0;
    int odON = 0;
    int poseStatON = 0;
    int saShedON = 0;
    int excSqON = 0;
    int distON = 0;
    int posePalletON = 0;

    int arduinoON = 0;
    int mThreadsON = 0;

    int avoidOpponent = 0;
    int contest = 1;
    int started = 0;

    int lidar_caract = 0;

    auto begin_time = high_resolution_clock::now();
    arduino_send(cvs,"R");

    //set_initial_time(cvs);
    if (contest) {
        printf("let's go!\n");
        get_d2r_data(cvs);
        arduino_send(cvs,"R");
        cvs->mp->x = 3-0.133;
        cvs->mp->y = 1.467;
        cvs->mp->th = M_PI;
        printf("team = %d \n",inputs->team);
        if(!inputs->team){
            cvs->mp->x = .133;
            cvs->mp->th = 0;
            teensy_send(cvs,"2");
        } else teensy_send(cvs, "1");
        threads_start(cvs);
        teensy_send(cvs, "Q");

        double maxTime = 99.75;
        //if(cvs->inputs->option2) maxTime = 139.5;
        //changeSettings(cvs);

        while(inputs->t < maxTime){
            auto start = high_resolution_clock::now();

            auto tester_begin=high_resolution_clock::now();
            teensy_recv(cvs);
            auto tester_stop = high_resolution_clock::now();
            auto tester_duration = duration_cast<microseconds>(stop - start);
            fprintf("%f\n",tester_duration.count());

            if (!started) {
                get_d2r_data(cvs);
                //inputs->t = 0;
                //set_initial_time(cvs);
                begin_time = high_resolution_clock::now();
                if(inputs->start) {
                    started = 1; 
                    arduino_send(cvs,"1");
                    printf("START!\n");
                }
            }
            update_pos(cvs);
            if(started){
                strategy_loop(cvs);
            }

            update_time(cvs);
            auto stop = high_resolution_clock::now();
            auto duration = duration_cast<microseconds>(stop - start);
            usleep(dt * 1000000 - duration.count());

            stop = high_resolution_clock::now();
            duration = duration_cast<microseconds>(stop- begin_time );
            cvs->inputs->t = duration.count()*1e-6;

        }
        printf("ENDED\n");
        mt->thread_main_end = 1;
        printf("th_end : start ... ");
        threads_end(cvs);
    }
    if (lidar_caract) {
        threads_start(cvs);

        inputs->team = 0;

        cvs->mp->x = 3-0.133;
        cvs->mp->y = 1.467;
        cvs->mp->th = M_PI;

        while (rec->n_rec < 101) {
            //printf("------------- rec static -------------\n");
            rec_static(cvs);
        }
        usleep(500000);
        mt->thread_main_end = 1;
        printf("th_end : start ... ");
        threads_end(cvs);
    }
    if (avoidOpponent){
        threads_start(cvs);
        get_d2r_data(cvs);
        //inputs->team = 1;
        saShed_launch(cvs);
        printf("avoidopp\n");
        cvs->mp->x = 3-0.133;
        cvs->mp->y = 1.467;
        cvs->mp->th = M_PI;
        printf("team = %d \n",inputs->team );
        if(!inputs->team){
            teensy_send(cvs, "2");
            cvs->mp->x = .133;
            cvs->mp->th = 0;
        }
        else teensy_send(cvs, "1");
        double xgoal = cvs->mp->x;double ygoal=cvs->mp->y;
        int forward;double orientation;
        set_param_normal(cvs);
        //inputs->team = 1;
        cvs->mp->x = 3-0.14;
        cvs->mp->y = 2-0.53;
        cvs->mp->th = M_PI;

        printf("begin test hlcPavoid oppoent \n");
        while (inputs->t < 75) {
            auto start = high_resolution_clock::now();
            double t = inputs->t;
            if (t==0) {
                xgoal = 1;//2.2;//1.2;
                ygoal = 1;//1.60;
                forward=-1;
                orientation = M_PI;//M_PI/2;
                set_goal(cvs, xgoal, ygoal, orientation);
                printf("goal A\n");
            } else if (t>25 & t<25.01) {
                //set_param_prec(cvs);
                xgoal = 2;
                ygoal = 1;
                forward =-1;
                orientation = M_PI;
                set_goal(cvs, xgoal, ygoal, orientation);
                printf("goal B\n");
            } else if (t>50 & t<50.1) {
                xgoal = 1;//1.3;
                ygoal = 1;
                forward = -1;
                orientation = M_PI;
                set_goal(cvs, xgoal, ygoal, orientation);
                printf("goal c\n");
            }
            //get_d2r_data(cvs); // ctrlIn
            sendFromHLCPF(cvs,forward);

            //set_new_position(cvs);
            //printf("cmd_r = %d | cmd_l = %d\n", outputs->r_cmd, outputs->l_cmd);
            //printf("x = %f | y = %f | th = %f\n", mp->x, mp->y, mp->th);

            fprintf(cvs->llc_data, "%f,%f,%f,%f,%f,%f,%f\n", inputs->t, mlcPF->r_sp_ref, mlcPF->l_sp_ref, inputs->r_sp_mes_enc, inputs->l_sp_mes_enc, inputs->r_sp_mes_odo, inputs->l_sp_mes_odo);

            update_time(cvs);
            auto stop = high_resolution_clock::now();
            auto duration = duration_cast<microseconds>(stop - start);

            //printf("duration.count() = %lld us | t = %f\n-------------\n", duration.count(), inputs->t);
            usleep(dt * 1000000 - duration.count());
        }
    }

    if (odON) {
        threads_start(cvs);

        inputs->team = 0;

        cvs->mp->x = 0.3;
        cvs->mp->y = 0.75;
        cvs->mp->th = 0;

        printf("------------- rec static -------------\n");
        while (1) {
            if (rec_static(cvs)) break;
            dyn_obs_set(cvs);
        }
        usleep(1000000);
        printf("----------------- od -----------------\n");
        while (1) {
            if (od_distrib(cvs, 0, 0) > -10) break;
            dyn_obs_set(cvs);
        }
        usleep(1000000);

        mt->thread_main_end = 1;
        printf("th_end : start ... ");
        threads_end(cvs);
        printf("end\n");
    }
    if (icpDynON) {
        threads_start(cvs);

        cvs->mp->x = 3-0.14;//2.00;//3-0.14;
        cvs->mp->y = 1.13;//0.75;//0.445+0.125;//0.45;//2-0.53;
        cvs->mp->th = M_PI;//0;//M_PI;

        double v_ref = 0.2;
        double th_ref = M_PI;

        //op->no_opp = 1;
        int i = 0;
        while (inputs->t < 3) {
            auto start = high_resolution_clock::now();

            sendFromMLCPF(cvs, v_ref, th_ref);

            if (rec_ON(cvs)) {
                printf("rec #%d : end\n", i++);
            }
            /*
            if (inputs->t >= 1.5 && i == 0) {
                if (rec_ON(cvs)) {
                    printf("rec #%d : end\n", i++);
                }
            }
             */
            update_time(cvs);
            auto stop = high_resolution_clock::now();
            auto duration = duration_cast<microseconds>(stop - start);
            usleep(dt * 1000000 - duration.count());
        }

        printf("------------- rec static -------------\n");
        rec->iter = 0;
        motors_stop(cvs);
        while (1) if (rec_static(cvs)) break;
        usleep(2000000);

        mt->thread_main_end = 1;
        printf("th_end : start ... ");
        threads_end(cvs);
        printf("end\n");
    }
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
        while (rpl->nTurns < 5) {
            printf("rpl->nTurns : %d\n", rpl->nTurns);
            rpl_grabData(cvs);
            if (rpl->nTurns == 3) {
                printf("ICP go\n");
                rec_ICP(cvs, &icp);
                break;
            }

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
        cvs->mp->y = 1.13;//2-0.53;//0.795+0.125;//1.13;//0.45;//2-0.53;
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
        get_d2r_data(cvs);
        cvs->mp->x = 3-0.133;
        cvs->mp->y = 1.13;
        cvs->mp->th = M_PI;
        if(!inputs->team){
            cvs->mp->x = .133;
            cvs->mp->th = 0;
        }

        mlcPF->t_start = inputs->t;
        while (inputs->t < 10000) {
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

        while (inputs->t < 5.1) {
            auto start = high_resolution_clock::now();

            get_d2r_data(cvs); // ctrlIn

            printf("r_sp_mes_enc = %f | l_sp_mes_enc = %f\n", inputs->r_sp_mes_enc, inputs->l_sp_mes_enc);
            //printf("r_sp_mes_odo = %f | l_sp_mes_odo = %f\n", inputs->r_sp_mes_odo, inputs->l_sp_mes_odo);

            if (inputs->t >= 0 && inputs->t < 5) {
                r_cmd = 10;
                l_cmd = 10;
            }
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
        teensy_send(cvs,"I");

        while (inputs->t < 2) {
            auto start = high_resolution_clock::now();

            get_d2r_data(cvs); // ctrlIn

            printf("r_sp_mes_enc = %f | l_sp_mes_enc = %f\n", inputs->r_sp_mes_enc, inputs->l_sp_mes_enc);
            printf("r_sp_mes_odo = %f | l_sp_mes_odo = %f\n", inputs->r_sp_mes_odo, inputs->l_sp_mes_odo);

            if (inputs->t >= 0 && inputs->t < 1) {
                r_sp_ref = 10;
                l_sp_ref = 10;
            }

            else if (inputs->t >= 1 && inputs->t < 2) {
                r_sp_ref = 0;
                l_sp_ref = 0;
            }
            else if (inputs->t >= 4 && inputs->t < 7) {
                r_sp_ref = -10;
                l_sp_ref = -10;
            }
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

            // to retrieve
            //update_time(cvs);
            auto stop = high_resolution_clock::now();
            auto duration = duration_cast<microseconds>(stop - start);
            printf("duration.count() = %lld us | t = %f\n-------------\n", duration.count(), inputs->t);
            usleep(dt * 1000000 - duration.count());

            //to add !!
            stop = high_resolution_clock::now();
            duration = duration_cast<microseconds>(stop- begin_time );
            cvs->inputs->t = duration.count()*1e-6;

            printf("duration %f\n", cvs->inputs->t);

        }
    }
    if (mlcPF_ON) {

        mp->th = 0;
        double v_ref = 0.2;
        double th_ref;
        cvs->mp->x = 3-0.14;//3-0.94;//3-0.14;
        cvs->mp->y = 1.13;//2-0.53;//0.795+0.125;//1.13;//0.45;//2-0.53;
        cvs->mp->th = 0;//0;//M_PI;

        mlcPF->t_start = inputs->t;
        while (inputs->t < 12) {
            if(inputs->t<2) {
                v_ref = 0;
                th_ref = 0;
            }
            else if (inputs->t >= 2 && inputs->t < 7) {
                v_ref = .30;
                th_ref = M_PI*.9;
            }

            else if (inputs->t >= 7 && inputs->t < 15) {
                v_ref = 0.3;
                th_ref = M_PI/2;
            }
            else if (inputs->t >= 10 && inputs->t < 13) {
                v_ref = 0.3;
                th_ref = -M_PI/2;
            }
            else {
                v_ref = 0.01;
                th_ref = M_PI/4;
            }
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
            fprintf(cvs->mlc_data, "%f,%f,%f,%f,%f\n",inputs->t,v_ref,th_ref,mp->v,mp->th );

            //fprintf(cvs->mlcPF, "%f,%f,%f\n", );
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

            int data = d2r_enc_measure(cvs,-1, 0, 2 ,true);

            r_ticks_enc_tot += r_ticks_enc;
            l_ticks_enc_tot += l_ticks_enc;
            r_ticks_odo_tot += r_ticks_odo;
            l_ticks_odo_tot += l_ticks_odo;


            fprintf(cvs->llc_data, "%d\n",r_ticks_enc);
            //fprintf(cvs->llc_data, "%f,%d,%d,%d,%d\n",inputs->t,r_ticks_enc,l_ticks_enc,r_ticks_odo,l_ticks_odo);

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
    if (checkRepetabilityHLCPFTrianlge) {

        threads_start(cvs);
        get_d2r_data(cvs);
        printf("checkRepetabilityHLCPF\n");
        cvs->mp->x = 3-1.15;
        cvs->mp->y = 0.445+.125;
        cvs->mp->th = M_PI;
        printf("team = %d \n",inputs->team );
        set_param_normal(cvs);
        int size = 3;
        double xs[size] = {1.15,1.15,1.85};
        double ys[size] = {.57,1.18,.57};
        int pt = 0;
        printf("begin test hlcPF\n");
        set_goal(cvs,xs[pt%size],ys[pt%size],-10);

        int rec = 0;
        double pos[5];
        double x_norec, y_norec, th_norec;

        while (inputs->t < 300) {
            auto start = high_resolution_clock::now();
            update_pos(cvs);
            double t = inputs->t;
            set_param_normal(cvs);
            if(rec){
                if(checkChrono(cvs)){
                    if(rec_static(cvs)|checkChrono(cvs,1)){
                        rec = 0;
                    }
                } else {
                    setChrono(cvs,1,1);
                }
            } else {
                sendFromHLCPF(cvs,1,1);
                if(hlcPF->output){
                    pt+=1;
                    double xg = xs[pt%size];
                    double yg = ys[pt%size];
                    set_goal(cvs,xg,yg,-10);
                    rec = 1;
                    get_pos(cvs, pos);
                    th_norec = pos[2];
                    x_norec = pos[0];
                    y_norec = pos[1];
                    fprintf(cvs->caract_odo_data,"%f,%f,%f,%f\n",inputs->t,x_norec,y_norec,th_norec);
                    printf("positions : %f,%f,%f\n",x_norec,y_norec,th_norec);
                    setChrono(cvs,1);
                }
            }
            //fprintf(cvs->llc_data, "%f,%f,%f,%f,%f,%f,%f\n", inputs->t, mlcPF->r_sp_ref, mlcPF->l_sp_ref, inputs->r_sp_mes_enc, inputs->l_sp_mes_enc, inputs->r_sp_mes_odo, inputs->l_sp_mes_odo);
            update_time(cvs);
            auto stop = high_resolution_clock::now();
            auto duration = duration_cast<microseconds>(stop - start);
            usleep(dt * 1000000 - duration.count());
        }

        mt->thread_main_end = 1;
        printf("th_end : start ... ");
        threads_end(cvs);
        printf("end\n");
        //mt->thread_main_end = 1;

        //threads_end(cvs);
    }

    if (odo_caracterisation_ON) {

        threads_start(cvs);
        get_d2r_data(cvs);
        printf("checkRepetabilityHLCPF\n");
        cvs->mp->x = .5;
        cvs->mp->y = 1;
        cvs->mp->th = 0;
        printf("team = %d \n",inputs->team );
        set_param_normal(cvs);
        int size = 8;
        double xs[size] = {.5,1,1.5,2,  2.5,2,1.5,1,.5};
        double ys[size] = {1,1,1,1,     1,1,1,1};
        printf("begin test hlcPF\n");

        int pt = 1;
        set_goal(cvs,xs[pt%size],ys[pt%size],-10);

        int rec = 0;
        int orient = 0;
        int rec_change = 0;

        double pos[5];
        double x_norec, y_norec, th_norec;

        //FAUX POUR LE MOMENT
        while (inputs->t < 100) {
            auto start = high_resolution_clock::now();
            update_pos(cvs);
            double t = inputs->t;
            set_param_normal(cvs);

            if(rec){
                if(checkChrono(cvs)){
                    if(rec_static(cvs)|checkChrono(cvs,1)){//retrieve the 1 in rec_static
                        rec = 0;
                    }
                } else {
                    setChrono(cvs,1,1);
                }
            }
            else if(orient){
                get_pos(cvs, pos);
                th_norec = pos[2];
                x_norec = pos[0];
                y_norec = pos[1];
                double orientation = 0;
                if(pt==5){orientation = M_PI;}
                //cvs->mlcPF->K
                sendFromMLCPF(cvs,0,orientation);
                if(abs(limit_angle(th_norec - orientation)) < 0.15){
                    orient = 0;
                }
            }

            else if(rec_change){
                if(checkChrono(cvs)){
                    if(rec_static(cvs,1)|checkChrono(cvs,1)){
                        rec_change = 0;
                    }
                } else {
                    setChrono(cvs,1,1);
                }

            } else {
                set_param_prec(cvs);
                sendFromHLCPF(cvs,1,1);
                if(hlcPF->output){
                    pt+=1;
                    pt=pt%size;
                    double orientation = 0;
                    if(pt>=4){
                        orientation = M_PI;
                    } 
                    double xg = xs[pt];
                    double yg = ys[pt];
                    set_goal(cvs,xg,yg,orientation);

                    rec = 1;
                    rec_change = 0;orient = 0;
                    if(pt == 5 |pt ==1){
                        //rec_change = 1;
                        orient = 1;
                    }

                    get_pos(cvs, pos);
                    th_norec = pos[2];
                    x_norec = pos[0];
                    y_norec = pos[1];
                    fprintf(cvs->caract_odo_data,"%f,%f,%f,%f\n",inputs->t,x_norec,y_norec,th_norec);
                    printf("positions : %f,%f,%f\n",x_norec,y_norec,th_norec);
                    setChrono(cvs,1);
                }
            }
            //fprintf(cvs->llc_data, "%f,%f,%f,%f,%f,%f,%f\n", inputs->t, mlcPF->r_sp_ref, mlcPF->l_sp_ref, inputs->r_sp_mes_enc, inputs->l_sp_mes_enc, inputs->r_sp_mes_odo, inputs->l_sp_mes_odo);
            update_time(cvs);
            auto stop = high_resolution_clock::now();
            auto duration = duration_cast<microseconds>(stop - start);
            usleep(dt * 1000000 - duration.count());
        }
        mt->thread_main_end = 1;
        printf("th_end : start ... ");
        threads_end(cvs);
        printf("end\n");
    }


    if (hlcPFON) {

        threads_start(cvs);

        get_d2r_data(cvs);
        saShed_launch(cvs);
        printf("hlcPFON\n");
        cvs->mp->x = 3-0.133;
        cvs->mp->y = 1.467;
        cvs->mp->th = M_PI;
        printf("team = %d \n",inputs->team );
        if(!inputs->team){
            teensy_send(cvs, "2");
            cvs->mp->x = .133;
            cvs->mp->th = 0;
        }
        else teensy_send(cvs, "1");
        double xgoal = cvs->mp->x;double ygoal=cvs->mp->y;
        int forward;double orientation;
        set_param_normal(cvs);

        printf("begin test hlcPF\n");
        while (inputs->t < 80) {
            auto start = high_resolution_clock::now();
            update_pos(cvs);
            double t = inputs->t;
            if (t<=.1) {
                set_param_normal(cvs);
                xgoal = 1;//2.2;//1.2;
                ygoal = 1.55;//1.60;
                forward = -1;
                orientation = -10;//M_PI/2;
                set_goal(cvs, xgoal, ygoal, orientation);
                printf("goal A\n");
            } else if (t>20 & t<20.01) {
                //set_param_prec(cvs);
                xgoal = .75;
                ygoal = .5;
                forward =-1;
                orientation = -10;
                set_goal(cvs, xgoal, ygoal, orientation);
                printf("goal B\n");
            } else if (t>30 & t<30.1) {
                xgoal = 2.5;//1.3;
                ygoal = 1.4;
                forward = -1;
                orientation = 0;
                set_goal(cvs, xgoal, ygoal, orientation);
                printf("goal c\n");
            }
            //get_d2r_data(cvs); // ctrlIn
            if(!hlcPF->output) sendFromHLCPF(cvs,forward);
            //set_new_position(cvs);
            //printf("cmd_r = %d | cmd_l = %d\n", outputs->r_cmd, outputs->l_cmd);
            //printf("x = %f | y = %f | th = %f\n", mp->x, mp->y, mp->th);

            fprintf(cvs->llc_data, "%f,%f,%f,%f,%f,%f,%f\n", inputs->t, mlcPF->r_sp_ref, mlcPF->l_sp_ref, inputs->r_sp_mes_enc, inputs->l_sp_mes_enc, inputs->r_sp_mes_odo, inputs->l_sp_mes_odo);

            update_time(cvs);
            auto stop = high_resolution_clock::now();
            auto duration = duration_cast<microseconds>(stop - start);

            //printf("duration.count() = %lld us | t = %f\n-------------\n", duration.count(), inputs->t);
            usleep(dt * 1000000 - duration.count());


        }
        mt->thread_main_end = 1;
        printf("th_end : start ... ");
        threads_end(cvs);
        printf("end\n");
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
            update_pos(cvs);
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
            update_pos(cvs);
            if(inputs->l_front_s < 5 | inputs->r_front_s<5){
                motors_stop(cvs);
            }
            else {
                pushShed_loop(cvs);
            }
            if(pshed->output) {
                printf("ended\n");
                //motors_stop(cvs);
                break;
            }
            update_time(cvs);
            auto stop = high_resolution_clock::now();
            auto duration = duration_cast<microseconds>(stop - start);
            usleep(dt * 1000000 - duration.count());
        }
    }
    if (saShedON){
        threads_start(cvs);

        teensy_send(cvs, "B");
        usleep(200000);
        teensy_send(cvs, "Q");
        usleep(200000);
        teensy_send(cvs, "R");
        usleep(200000);

        /*
        printf("------------- rec static -------------\n");
        rec->iter = 0;
        motors_stop(cvs);
        while (1) if (rec_static(cvs)) break;
         */
        //usleep(2000000);
        get_d2r_data(cvs);
        //inputs->team = 1;
        saShed_launch(cvs);
        printf("sasShedON\n");
        cvs->mp->x = 3-0.133;
        cvs->mp->y = 1.467;
        cvs->mp->th = M_PI;
        printf("team = %d \n",inputs->team );
        if(!inputs->team){
            teensy_send(cvs, "2");
            cvs->mp->x = .133;
            cvs->mp->th = 0;
        }
        else teensy_send(cvs, "1");

        while(inputs->t < 30){
            auto start = high_resolution_clock::now();
            teensy_recv(cvs);

            update_pos(cvs);
            saShed_loop(cvs);
            if(saShed->output) {
                printf("ended with %d\n",saShed->output);
                motors_stop(cvs);
                break;
            }
            fprintf(cvs->llc_data, "%f,%f,%f,%f,%f,%f,%f\n", inputs->t, mlcPF->r_sp_ref, mlcPF->l_sp_ref, inputs->r_sp_mes_enc, inputs->l_sp_mes_enc, inputs->r_sp_mes_odo, inputs->l_sp_mes_odo);
            update_time(cvs);
            auto stop = high_resolution_clock::now();
            auto duration = duration_cast<microseconds>(stop - start);
            usleep(dt * 1000000 - duration.count());
        }
        mt->thread_main_end = 1;
        printf("th_end : start ... ");
        threads_end(cvs);
        printf("end\n");
    }
    if (distON) {
        threads_start(cvs);

        teensy_send(cvs, "B");
        usleep(200000);
        teensy_send(cvs, "Q");
        usleep(200000);
        teensy_send(cvs, "R");
        usleep(200000);
        get_d2r_data(cvs);
        distr_launch(cvs);
        printf("distON\n");
        cvs->mp->x = 3-0.14;
        cvs->mp->y = 2-0.53;
        cvs->mp->th = M_PI;
        if(!inputs->team) {
            teensy_send(cvs, "2");
            cvs->mp->x = 0.14;
            cvs->mp->y = 2-0.53;
            cvs->mp->th = 0;
        }
        else teensy_send(cvs, "1");

        while(inputs->t < 15){
            auto start = high_resolution_clock::now();
            teensy_recv(cvs);

            update_pos(cvs);
            distr_loop(cvs,1);
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
        mt->thread_main_end = 1;
        printf("th_end : start ... ");
        threads_end(cvs);
        printf("end\n");
    }
    if (poseStatON){
        threads_start(cvs);

        get_d2r_data(cvs);

        teensy_send(cvs, "R");
        usleep(200000);
        teensy_send(cvs, "S");
        usleep(200000);
        /*
        printf("------------- rec static -------------\n");
        rec->iter = 0;
        motors_stop(cvs);
        while (1) if (rec_static(cvs)) break;
         */
        //usleep(2000000);
        poseStat_launch(cvs);
        printf("poseStat_launched\n");
        cvs->mp->x = 3-0.25;
        cvs->mp->y = .75;
        cvs->mp->th = M_PI;
        if(!inputs->team){
            teensy_send(cvs, "2");
            cvs->mp->x = .25;
            cvs->mp->th = 0;
        }
        else teensy_send(cvs, "1");

        while(inputs->t < 30){
            auto start = high_resolution_clock::now();
            teensy_recv(cvs);

            update_pos(cvs);
            poseStat_loop(cvs);
            if(poseStat->output) {
                printf("ended with %d\n",poseStat->output);
                motors_stop(cvs);
                break;
            }
            fprintf(cvs->llc_data, "%f,%f,%f,%f,%f,%f,%f\n", inputs->t, mlcPF->r_sp_ref, mlcPF->l_sp_ref, inputs->r_sp_mes_enc, inputs->l_sp_mes_enc, inputs->r_sp_mes_odo, inputs->l_sp_mes_odo);
            update_time(cvs);
            auto stop = high_resolution_clock::now();
            auto duration = duration_cast<microseconds>(stop - start);
            usleep(dt * 1000000 - duration.count());
        }
        mt->thread_main_end = 1;
        printf("th_end : start ... ");
        threads_end(cvs);
        printf("end\n");
    }

    if (posePalletON){
        threads_start(cvs);

        get_d2r_data(cvs);

        teensy_send(cvs, "R");
        pPallets_launch(cvs);
        printf("pPallets\n");
        cvs->mp->x = 3-0.133;
        cvs->mp->y = 1.13;
        cvs->mp->th = M_PI;

        cvs->distr->get = 1;
        if(!inputs->team){
            teensy_send(cvs, "2");
            cvs->mp->x = .133;
            cvs->mp->th = 0;
        }
        else teensy_send(cvs, "1");

        while(inputs->t < 30){
            auto start = high_resolution_clock::now();
            teensy_recv(cvs);

            update_pos(cvs);
            pPallets_loop(cvs,1);
            if(cvs->pPallets->output) {
                printf("ended with %d\n",cvs->pPallets->output);
                motors_stop(cvs);
                break;
            }
            fprintf(cvs->llc_data, "%f,%f,%f,%f,%f,%f,%f\n", inputs->t, mlcPF->r_sp_ref, mlcPF->l_sp_ref, inputs->r_sp_mes_enc, inputs->l_sp_mes_enc, inputs->r_sp_mes_odo, inputs->l_sp_mes_odo);
            update_time(cvs);
            auto stop = high_resolution_clock::now();
            auto duration = duration_cast<microseconds>(stop - start);
            usleep(dt * 1000000 - duration.count());
        }
        mt->thread_main_end = 1;
        printf("th_end : start ... ");
        threads_end(cvs);
        printf("end\n");
    }

    if (excSqON){
        threads_start(cvs);

        get_d2r_data(cvs);
        arduino_send(cvs,"R");
        teensy_send(cvs, "B");
        usleep(200000);
        teensy_send(cvs, "Q");
        usleep(200000);
        teensy_send(cvs, "R");
        /*
        printf("------------- rec static -------------\n");
        rec->iter = 0;
        motors_stop(cvs);
        while (1) if (rec_static(cvs)) break;
         */
        //usleep(2000000);
        excSq_launch(cvs);
        printf("excSqON\n");
        cvs->mp->x = 3-0.133;
        cvs->mp->y = 1.13;
        cvs->mp->th = M_PI;
        if(!inputs->team){
            teensy_send(cvs, "2");
            cvs->mp->x = .133;
            cvs->mp->th = 0;
        }
        else teensy_send(cvs, "1");
        while(inputs->t < 60){
            auto start = high_resolution_clock::now();
            teensy_recv(cvs);

            update_pos(cvs);
            excSq_loop(cvs);
            if(excSq->output) {
                printf("ended with %d\n",excSq->output);
                motors_stop(cvs);
                break;
            }
            fprintf(cvs->llc_data, "%f,%f,%f,%f,%f,%f,%f\n", inputs->t, mlcPF->r_sp_ref, mlcPF->l_sp_ref, inputs->r_sp_mes_enc, inputs->l_sp_mes_enc, inputs->r_sp_mes_odo, inputs->l_sp_mes_odo);
            update_time(cvs);
            auto stop = high_resolution_clock::now();
            auto duration = duration_cast<microseconds>(stop - start);
            usleep(dt * 1000000 - duration.count());
        }
        mt->thread_main_end = 1;
        printf("th_end : start ... ");
        threads_end(cvs);
        printf("end\n");
    }
    if (teensyON) {
        //teensy_send(cvs, "A");
        //usleep(1200000);
        //teensy_send(cvs, "Q");
        //usleep(1200000);
        //teensy_send(cvs, "R");
        teensy_send(cvs, "B");
        usleep(200000);
        //teensy_send(cvs, "Q");
        //usleep(1200000);
        teensy_send(cvs, "R");
        usleep(200000);
        teensy_send(cvs, "1");

        int A = 0;
        int B = 0;
        int C = 0;
        int D = 0;
        int K = 0;
        int Q = 0;
        int R = 0;
        int S = 0;
        while (inputs->t < 3) {

            auto start = high_resolution_clock::now();

            teensy_recv(cvs);

            //printf("switch_F : %d\n", teensy->switch_F);
            /*
            if (teensy->switch_F) {
                printf("Switch front ON\n");
                //teensy->switch_F = 0;
                teensy->switch_F_end = 1;
            }
            if (teensy->switch_B) {
                //teensy_send(cvs, "5");
                printf("Switch back ON\n");
                //teensy->switch_B = 0;
                teensy->switch_B_end = 1;
            }*/

            if (inputs->t >= 2 && C == 0) {
                teensy_send(cvs, "Q");
                C = 1;
            }

            if (inputs->t >= 4 && B == 0) {
                teensy_send(cvs, "S");
                B = 1;
            }

            if (inputs->t >= 8 && A == 0) {
                teensy_send(cvs, "Q");
                A = 1;
            }
            /*
            if (inputs->t >= 6 && D == 0) {
                teensy_send(cvs, "M");
                D = 1;
            }
             
            if (inputs->t >= 8 && R == 0) {
                teensy_send(cvs, "M");
                R = 1;
            }
            */
            update_time(cvs);
            auto stop = high_resolution_clock::now();
            auto duration = duration_cast<microseconds>(stop - start);
            //printf("duration.count() = %lld us | t = %f\n-------------\n", duration.count(), inputs->t);

            usleep(dt * 1000000 - duration.count());
        }
    }
    if (arduinoON) {
        arduino_send(cvs, "R");

        int one = 0;
        int two = 0;
        int three = 0;
        while (inputs->t < 5) {
            auto start = high_resolution_clock::now();

            teensy_recv(cvs);

            if (inputs->t >= 1 && one == 0) {
                one = 1;
                arduino_send(cvs, "1");
            }
            if (inputs->t >= 1.1 && two == 0) {
                two = 1;
                arduino_send(cvs, "K");
                arduino_send(cvs, "K");
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
    usleep(1200000);
    //teensy_send(cvs, "Q");
    //usleep(1200000);
    teensy_send(cvs, "R");
    usleep(1200000);
    cvs_free(cvs);

    return 0;
}

