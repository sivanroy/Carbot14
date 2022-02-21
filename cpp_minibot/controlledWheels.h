#ifndef CONTROLLEDWHEELS_H
#define CONTROLLEDWHEELS_H

#include <iostream>
#include <unistd.h>
#include <math.h>
#include <chrono>

#include "DualMC33926RPi.h"
#include "pid.h"

// 2pb: valeuress de vitesse très variable + valeur KI bizarre ...

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
		double Wdeltat = 0.01;
		double WMAX = 50;
		double WMIN = -WMAX;
		double WKP = 350; //400 seem's to be limit
		double WKD = 0;
		double WKI = WKP*0;
		double s_l;
		double s_r;
		DualMC33926RPi motors;
		PID leftPID =  PID(Wdeltat,WMAX,WMIN,WKP,WKD,WKI);
		PID rightPID =  PID(Wdeltat,WMAX,WMIN,WKP,WKD,WKI);
};

#endif
