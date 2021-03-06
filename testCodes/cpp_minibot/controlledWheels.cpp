#include "controlledWheels.h"

#ifndef CONTROLLEDWHEELS_SRC
#define CONTROLLEDWHEELS_SRC

double dt = 0.01;
double wheelP = 1;
double wheelI = 0.1;
double wheelD = 0;
double wheelMax = 50;
double wheelMin = -50;
double odoDia = 0.046;
double wheelDia = 0.06;
double ticksEnc = 1840;

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
	this->leftPID.FreeCache();
	this->rightPID.FreeCache();
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
	double backEMF = 0.0;//0.3;
	double v_l = this->leftPID.calculate(this->s_l,current_s_l) + backEMF * this->s_l;
	double v_r = this->rightPID.calculate(this->s_r,current_s_r) + backEMF * this->s_r;
	//printf("backemf %f , v_l %f \r\n",backEMF * this->s_l,v_l);
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