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

    /*!  rplidar data  */
    inputs->rpl_data_size = 0;
}

unsigned char d2r_enc_address(int encoder, int left, int sonar)
{
    unsigned char addr;

    if(encoder!=-1) {
        if (encoder) {
            if (left == 1) addr = 0x02;//0x04;//0x00; //0x01;
            else addr = 0x01;//0x01;//0x03; //0x00;
        } else {
            if (left == 1) addr = 0x04;//0x00;//0x02; //0x03;
            else addr = 0x03;//0x03;//0x01; //0x02;
        }
    }
    else {
        if(sonar == 0) {
            addr = 0x05;//0x05; //4
        }
        else if (sonar == 1){
            addr = 0x00;//0x02; //5
        }
    }
    return addr;
}

int d2r_enc_measure(ctrlStruct *cvs, int encoder, int left, int sonar,bool verbose = false)
{
    ctrlIn *inputs;
    inputs = cvs->inputs;

    unsigned char buffer[5];
    unsigned char addr = d2r_enc_address(encoder, left,sonar);
    buffer[0] = addr; buffer[1] = 0x00; buffer[2] = 0x00; buffer[3] = 0x00; buffer[4] = 0x00;

    wiringPiSPIDataRW(inputs->d2r_channel, buffer, 5);
    int count ;

    if(sonar == -1) count = buffer[4] + (buffer[3] << 8) + (buffer[2] << 16) + (buffer[1] << 24) - 4192;
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

    int d1 = d2r_enc_measure(cvs,-1,-1,0);
    int d2 = d2r_enc_measure(cvs,-1,-1,1);

    if(d1<300) inputs->l_front_s = d1;
    if(d2<300) inputs->r_front_s = d2;

    double r_sp_mes_enc = - r_ticks_enc * rpt_enc/dt;
    double l_sp_mes_enc =   l_ticks_enc * rpt_enc/dt;
    double r_sp_mes_odo = - r_ticks_odo * rpt_odo/dt;
    double l_sp_mes_odo =   l_ticks_odo * rpt_odo/dt;

    if (r_sp_mes_enc > -20 && r_sp_mes_enc < 20) inputs->r_sp_mes_enc = r_sp_mes_enc;
    if (l_sp_mes_enc > -20 && l_sp_mes_enc < 20) inputs->l_sp_mes_enc = l_sp_mes_enc;
    if (r_sp_mes_odo > -20 && r_sp_mes_odo < 20) inputs->r_sp_mes_odo = r_sp_mes_odo;
    if (l_sp_mes_odo > -20 && l_sp_mes_odo < 20) inputs->l_sp_mes_odo = l_sp_mes_odo;
}

void set_rpl_data(ctrlStruct *cvs)
{
    ctrlIn *inputs;
    rplStruct *rpl;

    inputs = cvs->inputs;
    rpl = cvs->rpl;

    int i;
    for (i = 0; i < rpl->data_size; i++) {
        inputs->rpl_a[i] = rpl->a[i];
        inputs->rpl_d[i] = rpl->d[i];
    }
    inputs->rpl_data_size = rpl->data_size;
}

void update_time(ctrlStruct *cvs)
{
    ctrlIn *inputs;
    inputs = cvs->inputs;

    inputs->t += inputs->dt;
}