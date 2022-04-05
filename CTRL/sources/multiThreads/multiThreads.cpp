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
    pthread_mutex_init(&(mt->mutex_rpl), NULL);

    mt->thread_main_end = 0;
}

int threads_start(ctrlStruct *cvs)
{
    mThreadsStruct *mt = cvs->mt;

    /*!  Threads launching  */
    int err = 0;

    err = pthread_create(&(mt->thread_op), NULL, &ctrl_op, cvs);
    if (err != 0) {
        perror("pthread_create(&thread_op) failed\n");
        return -1;
    }
    err = pthread_create(&(mt->thread_rec), NULL, &ctrl_rec, cvs);
    if (err != 0) {
        perror("pthread_create(&thread_rec) failed\n");
        return -1;
    }
    return 0;
}

int threads_end(ctrlStruct *cvs)
{
    mThreadsStruct *mt = cvs->mt;

    /*!  Threads stopping  */
    int err = 0;

    err = pthread_cancel(mt->thread_op);
    if (err != 0) {
        perror("pthread_cancel(thread_op) failed\n");
        return -1;
    }
    err = pthread_join(mt->thread_op, NULL);
    if (err != 0) {
        perror("pthread_join(thread_op) failed\n");
        return -1;
    }

    err = pthread_cancel(mt->thread_rec);
    if (err != 0) {
        perror("pthread_cancel(thread_rec) failed\n");
        return -1;
    }
    err = pthread_join(mt->thread_rec, NULL);
    if (err != 0) {
        perror("pthread_join(thread_rec) failed\n");
        return -1;
    }
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

    while (mt->thread_main_end == 0) {
        rpl_grabData(cvs);
        get_opp_pos(cvs);
    }
    return 0;
}

void *ctrl_rec(void *arg)
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

    while (mt->thread_main_end == 0) {
        rec_ICP(cvs);
        //printf("ctrl_rec\n");
    }

    return 0;
}

int mutex_destroy(ctrlStruct *cvs)
{
    mThreadsStruct *mt = cvs->mt;

    int err = 0;

    if (pthread_mutex_destroy(&(mt->mutex_op)) != 0) {
        perror("pthread_mutex_destroy(&(mt->mutex_op)) failed\n");
        err = 1;
    }
    if (pthread_mutex_destroy(&(mt->mutex_mp)) != 0) {
        perror("pthread_mutex_destroy(&(mt->mutex_mp)) failed\n");
        err = 1;
    }
    if (pthread_mutex_destroy(&(mt->mutex_rpl)) != 0) {
        perror("pthread_mutex_destroy(&(mt->mutex_rpl)) failed\n");
        err = 1;
    }
    if (err) return -1;
    return 0;
}
