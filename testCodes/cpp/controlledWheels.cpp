#include "controlledWheels.h"

#ifndef CONTROLLEDWHEELS_SRC
#define CONTROLLEDWHEELS_SRC


controlledWheels::controlledWheels()
{
	this->motors.init();
	/*
	this->leftPID = PID(dt,wheelMax,wheelMin,wheelP,wheelD,wheelI);
	this->rightPID = PID(dt,wheelMax,wheelMin,wheelP,wheelD,wheelI);
	 */
	this->s_l = 0;
	this->s_r = 0;
}

void controlledWheels::init()
{
	//nothing to do ?
}

void controlledWheels::setSpeed(double sl, double sr)
{
	this->s_l = sl;
	this->s_r = sr;
}

void controlledWheels::stop()
{
	this->s_l = 0; this->s_r = 0;
	this->motors.stop();
}

void controlledWheels::start()
{
	//nothing to do ?
}

void controlledWheels::giveV(double current_s_l, double current_s_r, int out[])
{
	double v_l = this->leftPID.calculate(this->s_l,current_s_l);
	double v_r = this->rightPID.calculate(this->s_r,current_s_r);
	out[0] = (int) v_l; out[1] = (int) v_r;
}

void controlledWheels::sendV(double current_s_l, double current_s_r, bool verbose)
{
	int out[2];
	giveV(current_s_l,current_s_r,out);
	this->motors.setSpeeds(out[0], out[1]);
	if (verbose) printf("vl = %d  vr = %d\n", out[0],out[1]);
}

#endif