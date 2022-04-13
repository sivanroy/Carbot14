/*!
 * \file teensyStruct.cpp
 * \brief communication with the teensy
 */

/*!
Communication RPI -> Teensy :
1 -> Pallet violet (470 Ohm)
2 -> Pallet jaune (1 kOhm)
3 -> Pallet rouge (4.7 kOhm)
4 -> Back switch ON : Flip the pallets (Activate dyna ID4)
5 -> Front switch ON : Push under the shed (Activate dyna ID5)
R -> Reset all global variables to 0 (undo)
A -> Both front servo OUT : hold pallets to push under the shed
B -> Both front servo IN : release pallets to push under the shed
C -> Measure resistance
D -> Push cube
F -> Take 1st pallet from stack and drop it at the bottom of expoisiton gallery (pos: 100)
G -> Take 2nd pallet from stack and drop it at the bottom of expoisiton gallery (pos: 160)
H -> Take 3rd pallet from stack and drop it at the bottom of expoisiton gallery (pos: 220)
J
I
K -> Lower flip to take pallets from distributor (pos: 255)
L -> Lift the pallets 90 degrees with the flip (pos: 590)
M -> Put the flip in initial position (pos: 755)
N -> Take 1st pallet from stack and drop it at the top of expoisiton gallery (pos: 100)
O -> Take 1st pallet from stack and drop it at the top of expoisiton gallery (pos: 160)
P -> Take 1st pallet from stack and drop it at the top of expoisiton gallery (pos: 220)
Q -> Clamp IN -> release the statuette
R -> Clamp with its widest opening
S -> Clamp OUT -> clamps the statuette
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

    teensy->switch_B = 0;
    teensy->switch_B_end = 0;

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

    const char *data5;
    const char *data4;
    const char *data1;
    const char *data2;
    const char *data3;
    char str[BUF_SIZE];

    int n = RS232_PollComport(teensy->port, teensy->str_recv, (int) BUF_SIZE);
    if(n > 0) {
        teensy->str_recv[n] = 0;
        printf("Received %i bytes: '%s'\n", n, (char *) teensy->str_recv);

        data5 = "5";
        data4 = "4";
        data1 = "1";
        data2 = "2";
        data3 = "3";
        strcpy(str, data5);
        if (strcmp((char *) teensy->str_recv, (char *) str) == 0) {
            teensy->switch_F = 1;
        }
        strcpy(str, data4);
        if (strcmp((char *) teensy->str_recv, (char *) str) == 0) {
            teensy->switch_B = 1;
        }
        strcpy(str, data1);
        if (strcmp((char *) teensy->str_recv, (char *) str) == 0) {
            printf("-----------------  R1  -----------------\n");
        }
        strcpy(str, data2);
        if (strcmp((char *) teensy->str_recv, (char *) str) == 0) {
            printf("-----------------  R2  -----------------\n");
        }
        strcpy(str, data3);
        if (strcmp((char *) teensy->str_recv, (char *) str) == 0) {
            printf("-----------------  R3  -----------------\n");
        }
    }
}