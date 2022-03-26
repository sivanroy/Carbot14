//
// Created by Thomas Brouwers on 28/02/2022.
//
#include <stdlib.h>
#include <stdio.h>
#include <unistd.h>
#include <string.h>
#include "Lib/rs232/rs232.h"
#define BUF_SIZE 1

int main()
{
    int i=0;
    int cport_nr=24; /* /dev/ttyUSB0 */  //<------- ici, la valeur par défaut sera surement à changer on retrouve le 16, pour ma part je devais mettre le 24 comme port.
    int bdrate=57600; /* 9600 baud */
    char mode[]={'8','N','1',0}; // 8 data bits, no parity, 1 stop bit
    char str_send[BUF_SIZE]; // send data buffer
    unsigned char str_recv[BUF_SIZE]; // recv data buffer
    strcpy(str_send, "5");

    if(RS232_OpenComport(cport_nr, bdrate, mode, 0))
    {
        printf("Can not open comport\n");
        return(0);
    }
    usleep(1000000);  /* waits 2000ms for stable condition */
    while(1)
    {
        //usleep(200000);  /* waits for reply 1000ms */
        int n = RS232_PollComport(cport_nr, str_recv, (int)BUF_SIZE);
        if(n > 0){
            str_recv[n] = 0;   /* always put a "null" at the end of a string! */
            printf("Received %i bytes: '%s'\n", n, (char *)str_recv);

            if (strcmp((char *)str_recv, (char *) str_send) == 0) {
                printf("Sent to Arduino: '%s'\n", str_send);
                RS232_cputs(cport_nr, str_send); // sends string on serial
                usleep(2000000);
            }
        }

        //RS232_cputs(cport_nr, str_send[i]); // sends string on serial
        //printf("Sent to Arduino: '%s'\n", str_send[i]);
    }
    return(0);
}