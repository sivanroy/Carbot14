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

+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

 /*
* Communication RPI -> Arduino:
* R : reset à 4 pts
* 1 -> +1 : + 1 point for each sample removed from a distributor on the team side (including the shared distributor and the work shed);
*           + 1 point for each sample inside the camp;
*           + 1 additional point for each revealed and sorted sample inside the camp;
* 2 -> +2 : + 2 points for installing the statuette on the pedestal during preparation time;
*           + 2 points for installing a display cabinet during preparation time;
* 3 -> +3 : + 3 points for each sample inside the gallery;
*           + 3 additional points for each revealed and sorted sample inside the gallery;
* 5 -> +5 : + 5 points for each revealed excavation square at the team’s colour;
*           + 5 points additional if a least a excavation square at the team’s colour is revealed, and the red square at the team’s side is not revealed;
*           + 5 points for each sample inside the work shed;
*           + 5 points if the statuette is missing from the pedestal at the end of the game;
*           + 5 additional points if the display cabinet is activated;
* 6 -> +6
* A -> +10 : + 10 points if the replica is on the pedestal at the end of the game;
* F -> +15 : + 15 points if the statuette is inside the display cabinet at the end of the game;
* K -> +20 : + 20 points if all the team robots are inside either the camp either the excavation site;
*/


#include "teensyStruct.h"


void teensy_init(teensyStruct *teensy)
{
    teensy->t_port = 24;
    teensy->a_port = 17;
    teensy->t_bdrate = 57600;
    teensy->a_bdrate = 57600;
    //teensy->mode = {'8','N','1',0};
    teensy->mode[0] = '8';
    teensy->mode[1] = 'N';
    teensy->mode[2] = '1';
    teensy->mode[3] = 0;

    if(RS232_OpenComport(teensy->t_port, teensy->t_bdrate, teensy->mode, 0))
    {
        printf("teensy : can not open teensy comport\n");
    }
    usleep(100000);
    /*
    if(RS232_OpenComport(teensy->a_port, teensy->a_bdrate, teensy->mode, 0))
    {
        printf("teensy : can not open arduino comport\n");
    }
    usleep(100000);
    */
    teensy->switch_F = 0;
    teensy->switch_F_end = 0;

    teensy->switch_B = 0;
    teensy->switch_B_end = 0;

    teensy->no_switch = 1;

    teensy->no_R = 0;
    teensy->R1 = 0;
    teensy->R2 = 0;
    teensy->R3 = 0;
    teensy->R_mes1 = 0;
    teensy->R_mes4 = 0;
    teensy->R_mes5 = 0;
    teensy->R_mes6 = 0;
    teensy->R_mes7 = 0;
}

void arduino_send(ctrlStruct *cvs, const char *data)
{
    teensyStruct *teensy;
    teensy = cvs->teensy;

    strcpy(teensy->str_send, data);
    RS232_cputs(teensy->a_port, teensy->str_send);
    printf("Sent to arduino: '%s'\n", teensy->str_send);
    //usleep(2000);
}

void teensy_send(ctrlStruct *cvs, const char *data)
{
    teensyStruct *teensy;
    teensy = cvs->teensy;

    strcpy(teensy->str_send, data);
    RS232_cputs(teensy->t_port, teensy->str_send);
    printf("Sent to teensy: '%s'\n", teensy->str_send);
    //usleep(2000);
}

void teensy_recv(ctrlStruct *cvs) {
    teensyStruct *teensy;
    teensy = cvs->teensy;

    const char *data5;
    const char *data4;
    const char *dataX;
    const char *data0;
    const char *data1;
    const char *data2;
    const char *data3;
    char str[BUF_SIZE];

    int n = RS232_PollComport(teensy->t_port, teensy->str_recv, (int) BUF_SIZE);
    if (n > 0) {
        teensy->str_recv[n] = 0;
        //printf("Received %i bytes: '%s'\n", n, (char *) teensy->str_recv);

        data5 = "5";
        data4 = "4";
        dataX = "X";
        data0 = "0";
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
        strcpy(str, dataX);
        if (strcmp((char *) teensy->str_recv, (char *) str) == 0) {
            teensy->no_switch = 0;
            teensy->switch_F = 0;
            teensy->switch_B = 0;
        }
        strcpy(str, data0);
        if (strcmp((char *) teensy->str_recv, (char *) str) == 0) {
            teensy->no_R = 1;
            printf("-----------------  R0  -----------------\n");
        }
        strcpy(str, data1);
        if (strcmp((char *) teensy->str_recv, (char *) str) == 0) {
            teensy->R1 = 1;
            printf("-----------------  R1  -----------------\n");
        }
        strcpy(str, data2);
        if (strcmp((char *) teensy->str_recv, (char *) str) == 0) {
            teensy->R2 = 1;
            printf("-----------------  R2  -----------------\n");
        }
        strcpy(str, data3);
        if (strcmp((char *) teensy->str_recv, (char *) str) == 0) {
            teensy->R3 = 1;
            printf("-----------------  R3  -----------------\n");
        }
    }
    /*
    int m = RS232_PollComport(teensy->a_port, teensy->str_recv, (int) BUF_SIZE);
    if (m > 0) {
        teensy->str_recv[m] = 0;
        printf("Received %i bytes from a: '%s'\n", n, (char *) teensy->str_recv);
    }
     */
}