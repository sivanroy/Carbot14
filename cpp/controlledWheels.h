#ifndef CONTROLLEDWHEELS_H
#define CONTROLLEDWHEELS_H

#include <iostream>
#include <unistd.h>
#include <math.h>
#include "DualMC33926RPi.h"
#include "pid.h"


class controlledWheels 
{
	public:
		//io_init();
		controlledWheels();
        void init();
		void setSpeed(double s_l, double s_r);
		void stop();
		void start();
		void giveV(double current_s_l,double current_s_r,int out[]);
		void sendV(double current_s_l,double current_s_r,bool verbose=0);
		double givedt();
		double givewheelDia();
		double giveradPerTickEnc();

	private:
		double s_l;
		double s_r;
		DualMC33926RPi motors;
		PID leftPID;
		PID rightPID;
		double dt;
		double wheelDia;
		double radPerTickEnc;
};


#endif
