#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <cmath> //for sqrt
#include <unistd.h>
#include <chrono>

#include "distributeur.h"
#include "FSMs_utils.h"



enum {S0_di,DpmtHLCPF1_di,recalibrate_di,OpenDis_di,rec1_di,DpmtMLC1_di,DpmtMLC2_di,GetSamples_di,DpmtHLCPFOut_di,goout_di};

void distr_init(distributeurs *distr){
    distr->status = S0_di;
    distr->output = 0;
    distr->go = 0;

    distr->getCenter = 0;
    distr->getHome = 0;
    distr->get = 0;
    //distr->let = 0;
    printf("distrib initialized\n");
}

void distr_launch(ctrlStruct *cvs){
	cvs->distr->go = 1;
	cvs->distr->status = S0_ps;
    cvs->distr->output = 0;
    cvs->distr->get = 0;
}

void distr_loop(ctrlStruct *cvs,int center){
	distributeurs *distr = cvs->distr;
    myPosition *mp = cvs->mp;
    oppPosition *op = cvs->op;
    ctrlIn  *inputs = cvs->inputs;
    highLevelCtrlPF *hlcPF = cvs->hlcPF;
    midLevelCtrlPF *mlcPF = cvs->mlcPF;
    midLevelCtrl *mlc = cvs->mlc;
    teensyStruct *teensy = cvs->teensy;
    rplStruct *rpl = cvs->rpl;
    reCalibStruct *rec = cvs->rec;

    set_param_normal(cvs);
    int TEAM = cvs->inputs->team;

    //only if usefull
    double pos[5];
    double x, y, th;
    get_pos(cvs, pos);
    th = pos[2];
    x = pos[0];//+ hlcPF->x_shift * cos(th);
    y = pos[1];//+ hlcPF->x_shift * sin(th);
    double wait = 0.6;

    set_param_normal(cvs);
    switch(distr->status){
        case S0_di:{
            if(distr->get) {
                break;
                printf("cannot get the pallet because already have some\n");
            }

    		distr->status = DpmtHLCPF1_di;
            if (TEAM) set_goal(cvs,2.5,.75,M_PI);
            else set_goal(cvs,.5,.75,0);
            if (center) {
                if (TEAM) set_goal(cvs,3-1.2,1.5,-M_PI/2);
                else set_goal(cvs,1.2,1.5,-M_PI/2);
            }
            printf("go to dpHLCPF1\n");
    		distr->go = 0;
            distr->output = 0;
            break;
        }

        case DpmtHLCPF1_di:{
    		sendFromHLCPF(cvs,-1);
        	if(hlcPF->output){
                printf("go to recalibrate_di\n");
                distr->status = recalibrate_di;
                setChrono(cvs,wait);
                set_commands(cvs,0,0);
        	}
        	break;
        }

        case recalibrate_di:{
            if (rec_static(cvs)) {
                distr->status = OpenDis_di;
                printf("rec END\n");
            }
            if(checkChrono(cvs)) {
                printf("pb with lidar rec\n");
                distr->status = OpenDis_di;
            }
            break;
        }

        case OpenDis_di: {
            teensy_send(cvs, "K");
            distr->status = DpmtMLC1_di;
            printf("go to dpmtmlc\n");
            if (TEAM) set_goal(cvs,2.75,.75,M_PI);
            else set_goal(cvs,0.25,.75,0);
            if (center) {
                if (TEAM) set_goal(cvs,3-1.35,1.75,-M_PI/2);
                else set_goal(cvs,1.35,1.75,-M_PI/2);
            }
            break;
        }

        case DpmtMLC1_di:{
            set_param_prec(cvs);
            sendFromHLCPF(cvs,0,1,1);
            if(hlcPF->output){
                distr->status = DpmtMLC2_di;
                if (TEAM) set_goal(cvs,2.9,.75,0);
                else set_goal(cvs,0.1,.75,0);
                if (center) {
                    if (TEAM) set_goal(cvs,3-1.35,1.9,M_PI);
                    else set_goal(cvs,1.35,1.9,0);
                }
                printf("go to dpmtmlc2\n");
                set_commands(cvs,0,0);
                setChrono(cvs,2);
            }
            break;
        }

        case DpmtMLC2_di:{
            set_param_prec(cvs);
            hlcPF->Tau_max = .15;
            hlcPF->Tau_min = .1;
            mlcPF->sigma = 0.5;
            sendFromHLCPF(cvs,0,1,1);
            if(teensy->switch_B){
                teensy->switch_B = 0;
                distr->status = GetSamples_di;
                //teensy_send(cvs,"J");
                teensy_send(cvs,"L");
                setChrono(cvs,0.1);
                printf("go to GetSamples_di\n");
                if (TEAM) set_goal(cvs,3-.35,.75,-10);
                else set_goal(cvs,.35,.75,-10);
                if (center) {
                    if (TEAM) set_goal(cvs,3-1.35,1.75,-10);
                    else set_goal(cvs,1.35,1.75,-10);
                }
            } else if (checkChrono(cvs)){
                distr->status = recalibrate_di;
                setChrono(cvs,wait);
            }
            break;
        }

        case GetSamples_di:{
            if(checkChrono(cvs)){
                distr->get = 1;

                if(center) distr->getCenter = 1;
                else distr->getHome = 1;
                distr->status = DpmtHLCPFOut_di;
                printf("go to DpmtOut_ps\n");
            }
            break;
        }

        case DpmtHLCPFOut_di: {
            set_param_prec(cvs);
            hlcPF->Tau_max = .1;
            hlcPF->Tau_min = .1;
            mlcPF->sigma = 0.5;
            sendFromHLCPF(cvs,-1,1,1);
            if(hlcPF->output){
                distr->status = S0_di;
                printf("end loop\n");
                teensy_send(cvs,"L");
                arduino_send(cvs,"3");
                if (center&0) {
                    if (TEAM) set_goal(cvs,3-1.2,1.5,-10);
                    else set_goal(cvs,1.2,1.5,-10);
                    setChrono(cvs,3);
                    distr->status = goout_di;
                }                
                else distr->output = 1;
            }
            break;
        }

        case goout_di:{
            sendFromHLCPF(cvs,1);
            if(hlcPF->output|checkChrono(cvs)){
                distr->output = 1;
                distr->status = S0_di;
                printf("ended di in t = %d\n\n----------\n",cvs->inputs->t);
            }
            break;
        }

        default:
            printf("probleme defautl value in FSM dis\n");
    }

}