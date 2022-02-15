#include "controlledWheels.h"

#ifndef CONTROLLEDWHEELS_SRC
#define CONTROLLEDWHEELS_SRC

#include <iostream>
#include <unistd.h>
#include "DualMC33926RPi.h"
#include "PidLib/pid.h"

#define deltaT 0.01
#define WheelP 70
#define WheelI 50
#define WheelD 0
#define WheelMax 50
#define WheelMin -50
#define odoD 0.046
#define wheelD 0.056
#define RadPerTick 0.001

controlledWheels::controlledWheels()
{
	//this->motors = DualMC33926RPi 
	this->motors.init();
	this->leftPID = PID(deltaT,WheelMax,WheelMin,WheelP,wheelD,WheelI);
	this->rightPID = PID(deltaT,WheelMax,WheelMin,WheelP,wheelD,WheelI);
	this->s_l = 0;
	this->s_r = 0;
}

controlledWheels::init()
{
	//nothing to do ?
}

controlledWheels::setSpeed(sl,sr)
{
	this->s_l = sl;
	this->s_r = sr;
}

controlledWheels::stop()
{
	this->s_l = 0; this->s_r = 0;
	this->motors.stop();
}

controlledWheels::start()
{
	//nothing to do ?
}

controlledWheels::giveV(current_s_l,current_s_r,out)
{
	s_l = this->s_l;
	s_r = this->s_r;
	double v_l = this->leftPID.calculate(s_l,current_s_l);
	double v_r = this->rightPID.calculate(s_r,current_s_r);
	out[0] = (int) v_l; out[1] = (int) v_r;
}

controlledWheels::sendV(current_s_l,current_s_r,verbose)
{
	int out[2];
	giveV(current_s_l,current_s_r,out,verbose)
	wheels.setSpeeds(out[0], out[1]);	
	if(verbose){printf("vl = %d  vr = %d\n", v_l,v_r);}
}


#endif