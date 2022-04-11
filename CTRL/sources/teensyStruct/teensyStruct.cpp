/*!
 * \file teensyStruct.cpp
 * \brief communication with the teensy
 */

#include "teensyStruct.h"


void teensy_init(teensyStruct *teensy)
{
    teensy->port = 24;
    teensy->bdrate = 57600;
    //teensy->mode = {'8','N','1',0};
    teensy->mode[0] = '8';
    teensy->mode[1] = 'N';
    teensy->mode[2] = '1';
    teensy->mode[3] = 0;

    if(RS232_OpenComport(teensy->port, teensy->bdrate, teensy->mode, 0))
    {
        printf("teensy : can not open comport\n");
    }
    usleep(500000);

    teensy->switch_F = 0;
    teensy->switch_F_end = 0;

}

void teensy_send(ctrlStruct *cvs, const char *data)
{
    teensyStruct *teensy;
    teensy = cvs->teensy;

    strcpy(teensy->str_send, data);
    RS232_cputs(teensy->port, teensy->str_send);
    printf("Sent to teensy: '%s'\n", teensy->str_send);
    //usleep(2000);
}

void teensy_recv(ctrlStruct *cvs)
{
    teensyStruct *teensy;
    teensy = cvs->teensy;

    const char *data;
    const char *dataR1;
    const char *dataR2;
    const char *dataR3;
    char str[BUF_SIZE];

    int n = RS232_PollComport(teensy->port, teensy->str_recv, (int) BUF_SIZE);
    if(n > 0) {
        teensy->str_recv[n] = 0;
        printf("Received %i bytes: '%s'\n", n, (char *) teensy->str_recv);

        data = "5";
        dataR1 = "1";
        dataR2 = "2";
        dataR3 = "3";
        strcpy(str, data);
        if (strcmp((char *) teensy->str_recv, (char *) str) == 0) {
            teensy->switch_F = 1;
        }
        strcpy(str, dataR1);
        if (strcmp((char *) teensy->str_recv, (char *) str) == 0) {
            printf("-----------------  R1  -----------------\n");
        }
        strcpy(str, dataR2);
        if (strcmp((char *) teensy->str_recv, (char *) str) == 0) {
            printf("-----------------  R2  -----------------\n");
        }
        strcpy(str, dataR3);
        if (strcmp((char *) teensy->str_recv, (char *) str) == 0) {
            printf("-----------------  R3  -----------------\n");
        }
    }
}