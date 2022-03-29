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
    pthread_mutex_init(&(mt->mutex_mp), NULL);

    mt->thread_main_end = 0;
}

int threads_launcher(ctrlStruct *cvs)
{
    /*!  Threads initialisation  */
    pthread_t thread_main;
    pthread_t thread_op;

    /*!  Threads launching  */
    int err = 0;

    err = pthread_create(&thread_main, NULL, &ctrl_main, cvs);
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

    if (cvs->mt->thread_main_end) {
        err = pthread_cancel(thread_op);
        if (err != 0) {
            perror("pthread_cancel(thread_op) failed\n");
            return -1;
        }
    }
    err = pthread_join(thread_op, NULL);
    if (err != 0) {
        perror("pthread_join(thread_op) failed\n");
        return -1;
    }
    return 0;
}

void *ctrl_main(void *arg)
{
    ctrlStruct *cvs = (ctrlStruct *) arg;

    ctrlIn  *inputs = cvs->inputs;
    ctrlOut *outputs = cvs->outputs;
    lowLevelCtrl *llc = cvs->llc;
    myPosition *mp = cvs->mp;
    midLevelCtrlPF *mlcPF = cvs->mlcPF;
    highLevelCtrlPF *hlcPF = cvs->hlcPF;
    rplStruct *rpl = cvs->rpl;
    pushShed *pshed = cvs->pshed;
    midLevelCtrl *mlc = cvs->mlc;
    oppPosition *op = cvs->op;
    mThreadsStruct *mt = cvs->mt;
    teensyStruct *teensy = cvs->teensy;
    double dt = inputs->dt;

    pushShed_launch(cvs);
    cvs->mp->x = 3-0.17;//3-0.14;
    cvs->mp->y = 0.75;//0.45;//2-0.53;
    cvs->mp->th = M_PI;//0;//M_PI;

    while (inputs->t < 20) {
        auto start = high_resolution_clock::now();

        //printf("-------------\nmain loop\n");
        esquive_loop(cvs);
        //dyn_obs_set(cvs);

        update_time(cvs);
        auto stop = high_resolution_clock::now();
        auto duration = duration_cast<microseconds>(stop - start);
        //printf("duration.count() = %lld us | t = %f\n-------------\n", duration.count(), inputs->t);

        usleep(dt * 1000000 - duration.count());
    }
    mt->thread_main_end = 1;
    return 0;
}

void *ctrl_op(void *arg)
{
    ctrlStruct *cvs = (ctrlStruct *) arg;

    ctrlIn  *inputs = cvs->inputs;
    ctrlOut *outputs = cvs->outputs;
    lowLevelCtrl *llc = cvs->llc;
    myPosition *mp = cvs->mp;
    midLevelCtrlPF *mlcPF = cvs->mlcPF;
    highLevelCtrlPF *hlcPF = cvs->hlcPF;
    rplStruct *rpl = cvs->rpl;
    pushShed *pshed = cvs->pshed;
    midLevelCtrl *mlc = cvs->mlc;
    oppPosition *op = cvs->op;
    mThreadsStruct *mt = cvs->mt;
    teensyStruct *teensy = cvs->teensy;
    double dt = inputs->dt;

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
    if (pthread_mutex_destroy(&(mt->mutex_mp)) != 0) {
        perror("pthread_mutex_destroy(&(mt->mutex_mp)) failed\n");
        err = 1;
    }
    if (err) return -1;
    return 0;
}
