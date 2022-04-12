#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <cmath> //for sqrt
#include <unistd.h>
#include <chrono>

#include "distributeur.h"
#include "FSMs_utils.h"



enum {S0_di,DpmtHLCPF1_di,recalibrate_di,OpenDis_di,rec1_di,DpmtMLC1_di,DpmtMLC2_di,GetSamples_di,DpmtHLCPFOut_di};

void distr_init(distributeurs *distr){
    distr->status = S0_di;
    distr->output = 0;
    distr->go = 0;

    int s = 4; //2.28 ;; 1.51
    double x_goalsI[s] = {2.5,2.75,3};
    double y_goalsI[s] = {0.75,.75,.75};
    double thetasI[s] = {-M_PI,-M_PI,-10}; //s
    double forwardI[s] = {-1,0,0,0};
    for (int i=0; i<s;i++) {
    	distr->x_goals[i] = x_goalsI[i];
    	distr->y_goals[i] = y_goalsI[i];
        distr->thetas[i] = thetasI[i];
        distr->forward[i] = forwardI[i];
    }
    printf("sashed initialized\n");
}

void distr_launch(ctrlStruct *cvs){
	cvs->distr->go = 1;
	cvs->distr->status = S0_ps;
    cvs->distr->output=0;
}


void distr_loop(ctrlStruct *cvs){
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

    double x = mp->x; double y = mp->y;
    set_param_normal(cvs);
    switch(distr->status){
        case S0_di:
        	if(distr->go){
        		distr->status = DpmtHLCPF1_di;
                set_goal(cvs,distr->x_goals[0],distr->y_goals[0],distr->thetas[0]);
        		printf("go to dpHLCPF1\n");
        		distr->go = 0;
                distr->output = 0;
        	}
        	else motors_stop(cvs);
            break;

        case DpmtHLCPF1_di:{
    		sendFromHLCPF(cvs,cvs->distr->forward[0]);
        	if(hlcPF->output){
                printf("go to recalibrate_di\n");
                op->no_opp = 1;
                distr->status = recalibrate_di;
        	}
        	break;
        }

        case recalibrate_di:{
            if (rec->rpl_nTurn_set == 0) {
                rec->rpl_nTurn = rpl->nTurns + 2;
                rec->rpl_nTurn_set = 1;
                printf("rec->rpl_nTurn : %d\n", rec->rpl_nTurn);
            }
            if (rpl->nTurns == rec->rpl_nTurn) {
                rec_ON(cvs);
                rec->rpl_nTurn_set = 0;
                distr->status = OpenDis_di;
                printf("go to OpenDis_di\n");
            }
            break;
        }

        case OpenDis_di: {
            teensy_send(cvs, "M");
            //inputs->t = inputs->t + 2;
            distr->status = DpmtMLC1_di;
            //teensy_send(cvs, "L");
            //inputs->t = inputs->t + 2;
            //distr->status = DpmtMLC1_di;
            printf("go to dpmtmlc\n");
            set_goal(cvs,distr->x_goals[1],distr->y_goals[1],distr->thetas[1]);
            break;
        }

        case DpmtMLC1_di:{
            //hlcPF->Tau_max = 5;
            //sendFromMLC(cvs,distr->x_goals[1],distr->y_goals[1],cvs->distr->forward[1]);
            set_param_prec(cvs);
            sendFromHLCPF(cvs,distr->forward[1],1);
            if(hlcPF->output){
                //distr->status = DpmtHLCPFOut_di;
                distr->status = DpmtMLC2_di;
                set_goal(cvs,distr->x_goals[2],distr->y_goals[2],distr->thetas[2]);
                printf("go to dpmtmlc2\n");
            }
            break;
        }

        case DpmtMLC2_di:{
            //sendFromMLC(cvs,distr->x_goals[2],distr->y_goals[2],cvs->distr->forward[2]);
            set_param_prec(cvs);
            sendFromHLCPF(cvs,distr->forward[2],1);
            if(hlcPF->output){
                distr->status = GetSamples_di;
                //set_goal(cvs,distr->x_goals[3],distr->y_goals[3]);
                distr->output = 1;
                printf("go to GetSamples_di\n");
            }

            break;
        }

        case GetSamples_di:{
            //sendFromHLCPF(cvs,cvs->distr->forward[1]);
            if(hlcPF->output){
                distr->status = DpmtHLCPFOut_di;
                printf("go to DpmtOut_ps\n");
            }
            break;
        }

        case DpmtHLCPFOut_di: {
            distr->status = S0_di;
            distr->output = 1;
            printf("end loop\n");
            break;
        }

        default:
            printf("probleme defautl value in FSM\n");
    }

}