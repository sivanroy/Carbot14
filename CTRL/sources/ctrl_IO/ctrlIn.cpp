/*!
 * \file ctrlIn.cpp
 * \brief Structures defining the inputs of the robot controller
 */

#include "ctrlIn.h"


void ctrlIn_init(ctrlIn *inputs)
{
    /*!  Time variables  */
    inputs->t = 0.0;
    inputs->dt = 0.01;

    /*!  Global variables  */
    inputs->radPerTick_enc = 2 * M_PI/(8192 * 19);
    inputs->radPerTick_odo = 2 * M_PI/8192;

    /*!  DE02Rpi communication  */
    inputs->d2r_channel = 0;
    inputs->d2r_speed = 500000;
    wiringPiSPISetup(inputs->d2r_channel, inputs->d2r_speed = 500000);

    /*!  Measured speeds of encoders and odometers  */
    inputs->r_sp_mes_enc = 0.0;
    inputs->l_sp_mes_enc = 0.0;
    inputs->r_sp_mes_odo = 0.0;
    inputs->l_sp_mes_odo = 0.0;

    inputs->r_front_s = 0.0;
    inputs->l_front_s = 0.0;
    inputs->r_back_s = 0.0;
    inputs->l_back_s = 0.0;

    inputs->team = -1;
    inputs->start = -1;
}

void lpf_init(lowPassFilter *lpf){
    lpf->value_re = 0;
    lpf->value_le = 0;
    lpf->value_ro = 0;
    lpf->value_lo = 0;
    lpf->beta = .85 ;
}

double lpf(ctrlStruct *cvs,double val,int select){
    double prev;
    double beta = cvs->lpf->beta;
    if(select == 0) prev = cvs->lpf->value_re;
    if(select == 1) prev = cvs->lpf->value_le;
    if(select == 2) prev = cvs->lpf->value_ro;
    if(select == 3) prev = cvs->lpf->value_lo;
    double  out = beta*prev + (1 - beta) * val;
    if(select == 0) cvs->lpf->value_re = out;
    if(select == 1) cvs->lpf->value_le = out;
    if(select == 2) cvs->lpf->value_ro = out;
    if(select == 3) cvs->lpf->value_lo = out;
    return out;
}

unsigned char d2r_enc_address(int encoder, int left, int sonarF)
{
    unsigned char addr;

    if(encoder!=-1) {
        if (encoder) {
            if (left == 1) addr = 0x02;
            else addr = 0x01;
        } else {
            if (left == 1) addr = 0x04;
            else addr = 0x03;
        }
    }
    else {
        if (sonarF == 2) {
            addr = 0x00;
        }
        else if(sonarF == 1) {
            if (left == 0) addr = 0x05;
            else addr = 0x06;
        }
        else {
            if (left == 0) addr = 0x07;
            else addr = 0x08;
        }
    }
    return addr;
}

int d2r_enc_measure(ctrlStruct *cvs, int encoder, int left, int sonarF, bool verbose = false)
{
    ctrlIn *inputs;
    inputs = cvs->inputs;

    unsigned char buffer[5];
    unsigned char addr = d2r_enc_address(encoder, left, sonarF);
    buffer[0] = addr; buffer[1] = 0x00; buffer[2] = 0x00; buffer[3] = 0x00; buffer[4] = 0x00;

    wiringPiSPIDataRW(inputs->d2r_channel, buffer, 5);
    int count ;
    //printf("%x,%x,%x,%x,%x\n", buffer[0], buffer[1], buffer[2], buffer[3], buffer[4]);
    fprintf(cvs->llc_data2,"%x,%x,%x,%x,%x\n", buffer[0], buffer[1], buffer[2], buffer[3], buffer[4]);
    if(sonarF == -1) {
        count = buffer[4] + (buffer[3] << 8) + (buffer[2] << 16);// + (buffer[1] << 24)
        if (buffer[1] >= 128) count += ((buffer[1] - 128) << 24) - pow(2,31);
        else count += buffer[1] << 24;
    }  
    else count = buffer[4] + (buffer[3] << 8) + (buffer[2] << 16) + (buffer[1] << 24) ;
    if (verbose) printf("DE02Rpi::measure %d -> count = %d\n",addr, count);
    return count;
}

void get_d2r_data(ctrlStruct *cvs)
{
    ctrlIn *inputs;
    inputs = cvs->inputs;

    double dt = inputs->dt;
    double rpt_enc = inputs->radPerTick_enc;
    double rpt_odo = inputs->radPerTick_odo;

    int r_ticks_enc = d2r_enc_measure(cvs, 1, 0, -1);
    int l_ticks_enc = d2r_enc_measure(cvs, 1, 1, -1);
    int r_ticks_odo = d2r_enc_measure(cvs, 0, 0, -1);
    int l_ticks_odo = d2r_enc_measure(cvs, 0, 1, -1);

    int d_sFR = d2r_enc_measure(cvs,-1,0,1);
    int d_sFL = d2r_enc_measure(cvs,-1,1,1);
    int d_sBR = d2r_enc_measure(cvs,-1,0,0);
    int d_sBL = d2r_enc_measure(cvs,-1,1,0);

    int data = d2r_enc_measure(cvs, -1, 0, 2);

    inputs->start = data%2;
    if(inputs->team == -1){
        inputs->team = (data>>1)%2;
        printf("TEAM %d\n",inputs->team);
    }

    if (d_sFR < 300) inputs->r_front_s = d_sFR;
    if (d_sFL < 300) inputs->l_front_s = d_sFL; 
    if (d_sBR < 300) inputs->r_back_s  = d_sBR;
    if (d_sBL < 300) inputs->l_back_s  = d_sBL;

    double r_sp_mes_enc = - r_ticks_enc * rpt_enc/dt;
    double l_sp_mes_enc =   l_ticks_enc * rpt_enc/dt;
    double r_sp_mes_odo = - r_ticks_odo * rpt_odo/dt;
    double l_sp_mes_odo =   l_ticks_odo * rpt_odo/dt;

    //if(r_sp_mes_enc > )

    double MAX_enc = 20;
    
    if (r_sp_mes_enc > -MAX_enc && r_sp_mes_enc < MAX_enc) inputs->r_sp_mes_enc = lpf(cvs,r_sp_mes_enc, 0);
    if (l_sp_mes_enc > -MAX_enc && l_sp_mes_enc < MAX_enc) inputs->l_sp_mes_enc = lpf(cvs,l_sp_mes_enc,1);
    if (r_sp_mes_odo > -MAX_enc*1.4 && r_sp_mes_odo < MAX_enc*1.4) inputs->r_sp_mes_odo = r_sp_mes_odo;
    if (l_sp_mes_odo > -MAX_enc*1.4 && l_sp_mes_odo < MAX_enc*1.4) inputs->l_sp_mes_odo = l_sp_mes_odo;

    fprintf(cvs->lpf_data,"%f,%f,%f,%f,%f,%f,%f,%f\n",r_sp_mes_enc,l_sp_mes_enc,r_sp_mes_odo,l_sp_mes_odo,
        inputs->r_sp_mes_enc,inputs->l_sp_mes_enc,inputs->r_sp_mes_odo,inputs->l_sp_mes_odo);
}

void update_time(ctrlStruct *cvs)
{
    ctrlIn *inputs;
    inputs = cvs->inputs;

    inputs->t += inputs->dt;
}