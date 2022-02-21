#ifndef CONTROLLEDWHEELS_H
#define CONTROLLEDWHEELS_H

#include <iostream>
#include <unistd.h>
#include <math.h>
#include <chrono>

#include "CAN.h"
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

	private:
		double s_l;
		double s_r;
		CAN motors;
		PID leftPID =  PID(0.01,30,-30,1,0,0);
		PID rightPID =  PID(0.01,30,-30,1,0,0);
};


#endif
