/*!
 * \file multiThreads.cpp
 * \brief multi-threads functions
 */

#include "multiThreads.h"


void mt_init(mThreadsStruct *mt)
{
    /*!  Mutex initialisation  */
    pthread_mutex_init(&(mt->mutex_op), NULL);
}

int threads_launcher(ctrlStruct *cvs)
{
    /*!  Threads initialisation  */
    pthread_t thread_main;
    pthread_t thread_op;

    /*!  Threads launching  */
    int err = 0;

    err = pthread_create(&thread_main, NULL, (void*) ctrl_main, (void*) cvs);
    if (err != 0) {
        perror("pthread_create(&thread_main) failed\n");
        return -1;
    }
    err = pthread_create(&thread_op, NULL, (void*) ctrl_op, (void*) cvs);
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

    err = pthread_cancel(thread_op);
    if (err != 0) {
        perror("pthread_cancel(thread_op) failed\n");
        return -1;
    }
    err = pthread_join(thread_op, NULL);
    if (err != 0) {
        perror("pthread_join(thread_op) failed\n");
        return -1;
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
