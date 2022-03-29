/*!
 * \file multiThreads.h
 * \brief multi-threads functions
 */

#ifndef CARBOT14_MULTITHREADS_H
#define CARBOT14_MULTITHREADS_H

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <iostream>
#include <unistd.h>
#include <errno.h>
#include <math.h>
#include <chrono>
#include <pthread.h>

#include "../ctrlStruct/ctrlStruct.h"


typedef struct mThreadsStruct
{
    pthread_mutex_t mutex_op;
    pthread_mutex_t mutex_mp;

    int thread_main_end;

} mThreadsStruct;

void mt_init(mThreadsStruct *mt);
int threads_launcher(ctrlStruct *cvs);
void *ctrl_main(void *arg);
void *ctrl_op(void *arg);
int mutex_destroy(ctrlStruct *cvs);

#endif
