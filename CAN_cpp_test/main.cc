#include <cstdio>

#include <wiringPiSPI.h>
#include <unistd.h>
#include "CAN.hh"
#include "SPI_CAN.hh"
#include "SPI.hh"


#define CAN_BR 125e3

int main()
{
	printf("hello world\n");
	printf("##############################################################################################################\n");
    	printf("\t\t\tWelcome to the Minibot project of the ELEME2002 class :)");
    	printf("##############################################################################################################\n");
    	printf("\t\t I'm Miss Sunshine, please take care of me !\n");
    	printf("\t\t Please do not interchange the chips on my tower/motor PWM boards !\n");
    	printf("\t\t Try to respect the C-file interface when programming me because\n \t\t it will be the same in the robotic project (Q2) !\n");


	CAN *can;
	can = new CAN(CAN_BR);
	can->configure();
	can->ctrl_led(1);
	usleep(50);

	can->ctrl_motor(1);
	usleep(50);

	can->push_PropDC(5, 5);
	usleep (1000000*3);
    /*
	can->motors_setup();
	can->motors_cmd(0x30, 0x30);

	usleep(1000000 * 2);
    can->motors_cmd(0x23, 0x23);
    */
    can->push_PropDC(0, 0);

    can->ctrl_led(0);
    usleep(50);

    can->ctrl_motor(0);
    usleep(50);
}

