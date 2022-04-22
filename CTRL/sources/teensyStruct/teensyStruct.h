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
    int t_port;
    int a_port;
    int t_bdrate;
    int a_bdrate;
    char mode[4];

    char str_send[BUF_SIZE];
    unsigned char str_recv[BUF_SIZE];

    int switch_F;
    int switch_F_end;

    int switch_B;
    int switch_B_end;

    int no_switch;

    int no_R;
    int R1;
    int R2;
    int R3;
    int R_mes1;
    int R_mes2;
    int R_mes3;
    int R_mes4;
    int R_mes5;
    int R_mes6;
    int R_mes7;

} teensyStruct;

void teensy_init(teensyStruct *teensy);
void arduino_send(ctrlStruct *cvs, const char *data);
void teensy_send(ctrlStruct *cvs, const char *data);
void teensy_recv(ctrlStruct *cvs);

#endif
