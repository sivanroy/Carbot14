/*!
 * \file multiThreads.cpp
 * \brief multi-threads functions
 */

#include "multiThreads.h"

using namespace std::chrono;

void mt_init(mThreadsStruct *mt)
{
    /*!  Mutex initialisation  */
    pthread_mutex_init(&(mt->mutex_op), NULL);

    mt->thread_main_end = 0;
}

int threads_launcher(ctrlStruct *cvs)
{
    printf("dt = %f\n", cvs->inputs->dt);

    /*!  Threads initialisation  */
    pthread_t thread_main;
    pthread_t thread_op;

    /*!  Threads launching  */
    int err = 0;

    err = pthread_create(&thread_main, NULL, &ctrl_main, cvs); //void *(*start_routine) (void*)
    if (err != 0) {
        perror("pthread_create(&thread_main) failed\n");
        return -1;
    }
    err = pthread_create(&thread_op, NULL, &ctrl_op, cvs);
    if (err != 0) {
        perror("pthread_create(&thread_op) failed\n");
        return -1;
    }

    /*!  Threads stopping  */
    err = pthread_join(thread_main, NULL);
    if (err != 0) {
        perror("pthread_join(thread_main) failed\n");
        return -1;
    }
    printf("cvs->mt->thread_main_end = %d\n", cvs->mt->thread_main_end);

    if (cvs->mt->thread_main_end) {
        err = pthread_cancel(thread_op);
        printf("t_cancel : %d\n", err);
        if (err != 0) {
            perror("pthread_cancel(thread_op) failed\n");
            return -1;
        }
    }
    printf("join ?\n");
    err = pthread_join(thread_op, NULL);
    if (err != 0) {
        perror("pthread_join(thread_op) failed\n");
        return -1;
    }
    printf("join end\n");
    return 0;
}

void *ctrl_main(void *arg)
{
    ctrlStruct *cvs = (ctrlStruct *) arg;

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
    dt = inputs->dt;
    pshed = cvs->pshed;
    mlc = cvs->mlc;
    op = cvs->op;
    mt = cvs->mt;
    teensy = cvs->teensy;
    dt = inputs->dt;

    pushShed_launch(cvs);
    printf("pushShedON\n");
    cvs->mp->x = 3-0.14;
    cvs->mp->y = 2-0.53;
    cvs->mp->th = M_PI;

    printf("dt = %f\n", inputs->dt);

    int r_cmd = 0;
    int l_cmd = 0;

    while (inputs->t < 10) {
        auto start = high_resolution_clock::now();

        /*
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

        else {
            r_cmd = 0;
            l_cmd = 0;
        }
        outputs->r_cmd = r_cmd;
        outputs->l_cmd = l_cmd;
        send_commands(cvs); // ctrlOut

        fprintf(cvs->llc_data, "%f,%d,%d,%f,%f\n", inputs->t, r_cmd, l_cmd, inputs->r_sp_mes_enc, inputs->l_sp_mes_enc);
        */
        //
        printf("main loop\n");
        esquive_loop(cvs);
        //

        update_time(cvs);
        auto stop = high_resolution_clock::now();
        auto duration = duration_cast<microseconds>(stop - start);
        printf("duration.count() = %lld us | t = %f\n-------------\n", duration.count(), inputs->t);

        usleep(dt * 1000000 - duration.count());
    }
    mt->thread_main_end = 1;
    return 0;
}

void *ctrl_op(void *arg)
{
    ctrlStruct *cvs = (ctrlStruct *) arg;

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
    dt = inputs->dt;
    pshed = cvs->pshed;
    mlc = cvs->mlc;
    op = cvs->op;
    mt = cvs->mt;
    teensy = cvs->teensy;
    dt = inputs->dt;

    while (1) {
        rpl_grabData(cvs);
        set_rpl_data(cvs);
        get_opp_pos(cvs);
    }
    return 0;
}

int mutex_destroy(ctrlStruct *cvs)
{
    mThreadsStruct *mt;
    mt = cvs->mt;

    int err = 0;

    if (pthread_mutex_destroy(&(mt->mutex_op)) != 0) {
        perror("pthread_mutex_destroy(&(mt->mutex_op)) failed\n");
        err = 1;
    }
    if (err) return -1;
    return 0;
}
