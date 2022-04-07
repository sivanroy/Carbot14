/*!
 * \file teensyStruct.h
 * \brief communication with the teensy
 */

#ifndef CARBOT14_TEENSYSTRUCT_H
#define CARBOT14_TEENSYSTRUCT_H

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <iostream>
#include <unistd.h>
#include <errno.h>
#include <math.h>

#include "../ctrlStruct/ctrlStruct.h"
#include "rs232/rs232.h"

#define BUF_SIZE 1



typedef struct teensyStruct
{
    int port;
    int bdrate;
    char mode[4];

    char str_send[BUF_SIZE];
    unsigned char str_recv[BUF_SIZE];

    int switch_F;
    int switch_F_end;

} teensyStruct;

void teensy_init(teensyStruct *teensy);
void teensy_send(ctrlStruct *cvs, const char *data);
void teensy_recv(ctrlStruct *cvs);

#endif
