#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <cmath> //for sqrt

#include "excavationSquare.h"
#include "FSMs_utils.h"


//4 possibilités 
//carrés peuvent avoir ete retounré par l'ennemi
//attention a l'ennemi


enum {S0_es,Dpmt1_es,Dpmt2_es,Dpmt1_prec_es,Check1_es,Dpmt2_prec_es,Check2_es,Dpmt3_prec_es,Check3_es,Dpmt4_prec_es,
        Check4_es,Dpmt5_prec_es,Check5_es,Dpmt6_prec_es,Check6_es,Dpmt7_prec_es,Check7_es,Out_es,
    Dpmt1_noR_es,Check1_noR_es,Dpmt2_noR_es,Check2_noR_es,Dpmt3_noR_es,Check3_noR_es,Dpmt4_noR_es,Check4_noR_es,
    Dpmt5_noR_es,Check5_noR_es,Dpmt6_noR_es,Check6_noR_es,Dpmt7_noR_es,
    rec1_es,rec2_es,rec3_es,rec4_es,rec5_es,rec6_es,rec7_es,
    rec1_noR_es,rec2_noR_es,rec3_noR_es,rec4_noR_es,rec5_noR_es,rec6_noR_es,rec7_noR_es};

void excSq_init(excSquares *excSq){
    excSq->status = S0_ps;
    excSq->output = 0;
    excSq->go = 0;
    printf("excSq initialized\n");
}

void excSq_launch(ctrlStruct *cvs){
	cvs->excSq->go = 1;
	cvs->excSq->status = S0_ps;
    cvs->excSq->output = 0;

    cvs->excSq->at_least_one = 0;
    for (int i = 0; i < 8; i++) cvs->excSq->pts_flag[i] = 0;
}


void excSq_loop(ctrlStruct *cvs){
	excSquares *excSq = cvs->excSq;
    myPosition *mp = cvs->mp;
    ctrlIn  *inputs = cvs->inputs;
    highLevelCtrlPF *hlcPF = cvs->hlcPF;
    midLevelCtrlPF *mlcPF = cvs->mlcPF;
    midLevelCtrl *mlc = cvs->mlc;
    teensyStruct *teensy = cvs->teensy;

    set_param_normal(cvs);
    int TEAM = cvs->inputs->team;

    //only if usefull
    double pos[5];
    double x, y, th;
    get_pos(cvs, pos);
    th = pos[2];
    x = pos[0];//+ hlcPF->x_shift * cos(th);
    y = pos[1];//+ hlcPF->x_shift * sin(th);


    if (TEAM) {
        switch (excSq->status) {
            case S0_es: {
                if (excSq->go) {
                    excSq->status = Dpmt1_es;
                    if (TEAM) set_goal(cvs, 2.53, 0.5, M_PI / 2);
                    else set_goal(cvs, .65, 0.5, M_PI / 2);
                    printf("go to dp1\n");
                    excSq->go = 0;
                }
                break;
            }

            case Dpmt1_es: {
                sendFromHLCPF(cvs, -1);
                if (hlcPF->output) {
                    motors_stop(cvs);
                    set_commands(cvs, 0, 0);
                    excSq->status = Dpmt2_es;
                    if (TEAM) set_goal(cvs, 2.5, 0.2, M_PI);
                    else set_goal(cvs, .68, 0.25, 0.8 * M_PI);
                }
                break;
            }

            case Dpmt2_es: {
                sendFromHLCPF(cvs, -1, 1);
                if (hlcPF->output) {
                    motors_stop(cvs);
                    set_commands(cvs, 0, 0);
                    excSq->status = rec1_es;
                    if (TEAM) set_goal(cvs, 2.45, 0.21, M_PI);
                    else set_goal(cvs, 0.6675 + 0.1, .21, M_PI);
                }
                break;
            }
            case rec1_es: {
                if (rec_static(cvs)) {
                    printf("rec1_es END : go to Dpmt1_prec_es\n");
                    excSq->status = Dpmt1_prec_es;
                }
                break;
            }
            case Dpmt1_prec_es: {
                //set_param_normal(cvs);
                set_param_prec(cvs);
                hlcPF->Tau_max = .15;
                hlcPF->Tau_min = .1;
                mlcPF->sigma = 0.5;
                sendFromHLCPF(cvs, -1, 1);
                if (hlcPF->output) {
                    motors_stop(cvs);
                    set_commands(cvs, 0, 0);
                    excSq->status = Check1_es;
                    setChrono(cvs, 2);
                    teensy_send(cvs, "C");
                    printf("go to Check1_es\n");
                    //excSq->output = 1;
                }
                break;
            }
            case Check1_es: {
                if (teensy->R1) {
                    teensy->R_mes1 = 1;
                    if (excSq->at_least_one == 0) {
                        excSq->at_least_one = 1;
                        arduino_send(cvs,"A");
                        excSq->pts_flag[1] = 1;
                    } else {
                        if (excSq->pts_flag[1] == 0) {
                            arduino_send(cvs,"5");
                            excSq->pts_flag[1] = 1;
                        }
                    }
                } else if (teensy->R3) {
                    teensy->R_mes1 = 3;
                }
                if (checkChrono(cvs)) {
                    teensy->no_R = 0;
                    teensy->R1 = 0;
                    teensy->R2 = 0;
                    teensy->R3 = 0;
                    excSq->status = rec2_es;
                    printf("go to rec2_es\n");
                    if (TEAM) set_goal(cvs, 2.265, 0.21, M_PI);
                    else set_goal(cvs, 0.8525 + 0.1, .21, M_PI);
                }
                break;
            }
            case rec2_es: {
                if (rec_static(cvs)) {
                    printf("rec2_es END : go to Dpmt2_prec_es\n");
                    excSq->status = Dpmt2_prec_es;
                }
                break;
            }
            case Dpmt2_prec_es: {
                //set_param_normal(cvs);
                set_param_prec(cvs);
                hlcPF->Tau_max = .15;
                hlcPF->Tau_min = .1;
                mlcPF->sigma = 0.5;
                sendFromHLCPF(cvs, -1, 1);
                if (hlcPF->output) {
                    motors_stop(cvs);
                    set_commands(cvs, 0, 0);
                    excSq->status = Check2_es;
                    teensy_send(cvs, "T");
                    setChrono(cvs, 2);
                    printf("go to Check2_es\n");
                }
                break;
            }
            case Check2_es: {
                if (excSq->at_least_one == 0) {
                    excSq->at_least_one = 1;
                    arduino_send(cvs,"A");
                    excSq->pts_flag[2] = 1;
                } else {
                    if (excSq->pts_flag[2] == 0) {
                        arduino_send(cvs,"5");
                        excSq->pts_flag[2] = 1;
                    }
                }
                if (checkChrono(cvs)) {
                    if (teensy->R_mes1 == 1) {
                        excSq->status = rec4_es;
                        printf("1 : go to rec4_es\n");
                        if (TEAM) set_goal(cvs, 1.895 - 0.01, 0.21, M_PI);
                        else set_goal(cvs, 1.2225 + 0.1, 0.21, M_PI);
                    } else if (teensy->R_mes1 == 3) {
                        excSq->status = rec3_es;
                        printf("3 : go to rec3_es\n");
                        if (TEAM) set_goal(cvs, 2.08 - 0.01, 0.21, M_PI);
                        else set_goal(cvs, 1.0375 + 0.1, .21, M_PI);
                    } else if (teensy->R_mes1 == 0 || teensy->R_mes1 == 2) {
                        excSq->status = rec3_noR_es;
                        printf("0 : go to rec3_noR_es\n");
                        if (TEAM) set_goal(cvs, 2.08 - 0.01, 0.21, M_PI);
                        else set_goal(cvs, 1.0375 + 0.1, .21, M_PI);
                    } else {
                        printf("R_mes1 : %d \n", teensy->R_mes1);
                    }
                }
                break;
            }
            case rec3_es: {
                if (rec_static(cvs)) {
                    printf("rec3_es END : go to Dpmt3_prec_es\n");
                    excSq->status = Dpmt3_prec_es;
                }
                break;
            }
            case Dpmt3_prec_es: {
                //set_param_normal(cvs);
                set_param_prec(cvs);
                hlcPF->Tau_max = .15;
                hlcPF->Tau_min = .1;
                mlcPF->sigma = 0.5;
                sendFromHLCPF(cvs, -1, 1);
                if (hlcPF->output) {
                    motors_stop(cvs);
                    set_commands(cvs, 0, 0);
                    excSq->status = Check3_es;
                    setChrono(cvs, 2);
                    teensy_send(cvs, "T");
                    printf("go to Check3_es\n");
                }
                break;
            }
            case rec3_noR_es: {
                if (rec_static(cvs)) {
                    printf("rec3_noR_es END : go to Dpmt3_noR_es\n");
                    excSq->status = Dpmt3_noR_es;
                }
                break;
            }
            case Dpmt3_noR_es: {
                //set_param_normal(cvs);
                set_param_prec(cvs);
                hlcPF->Tau_max = .15;
                hlcPF->Tau_min = .1;
                mlcPF->sigma = 0.5;
                sendFromHLCPF(cvs, -1, 1);
                if (hlcPF->output) {
                    motors_stop(cvs);
                    set_commands(cvs, 0, 0);
                    excSq->status = Check3_es;
                    setChrono(cvs, 2);
                    teensy_send(cvs, "C");
                    printf("go to Check3_es\n");
                }
                break;
            }
            case Check3_es: {
                if (excSq->at_least_one == 0) {
                    excSq->at_least_one = 1;
                    arduino_send(cvs,"A");
                    excSq->pts_flag[3] = 1;
                } else {
                    if (excSq->pts_flag[3] == 0) {
                        arduino_send(cvs,"5");
                        excSq->pts_flag[3] = 1;
                    }
                }
                if (checkChrono(cvs)) {
                    excSq->status = rec4_es;
                    printf("go to rec4_es\n");
                    if (TEAM) set_goal(cvs, 1.895 - 0.01, 0.21, M_PI);
                    else set_goal(cvs, 1.2225 + 0.1, 0.21, M_PI);
                }
                break;
            }
            case rec4_es: {
                if (rec_static(cvs)) {
                    printf("rec4_es END : go to Dpmt4_prec_es\n");
                    excSq->status = Dpmt4_prec_es;
                }
                break;
            }
            case Dpmt4_prec_es: {
                //set_param_normal(cvs);
                set_param_prec(cvs);
                hlcPF->Tau_max = .15;
                hlcPF->Tau_min = .1;
                mlcPF->sigma = 0.5;
                sendFromHLCPF(cvs, -1, 1);
                if (hlcPF->output) {
                    motors_stop(cvs);
                    set_commands(cvs, 0, 0);
                    excSq->status = Check4_es;
                    setChrono(cvs, 2);
                    teensy_send(cvs, "C");
                    printf("go to Check4_es\n");
                }
                break;
            }
            case Check4_es: {
                if (teensy->R1) {
                    teensy->R_mes4 = 1;
                    if (excSq->at_least_one == 0) {
                        excSq->at_least_one = 1;
                        arduino_send(cvs,"A");
                        excSq->pts_flag[4] = 1;
                    } else {
                        if (excSq->pts_flag[4] == 0) {
                            arduino_send(cvs,"5");
                            excSq->pts_flag[4] = 1;
                        }
                    }
                } else if (teensy->R2) {
                    teensy->R_mes4 = 2;
                } else if (teensy->no_R) {
                    teensy->R_mes4 = 0;
                }
                if (checkChrono(cvs)) {
                    teensy->no_R = 0;
                    teensy->R1 = 0;
                    teensy->R2 = 0;
                    teensy->R3 = 0;
                    if (teensy->R_mes4 == 1) {
                        excSq->status = rec7_es;
                        printf("1 : go to rec7_es\n");
                        if (TEAM) set_goal(cvs, 1.34 - 0.01, 0.21, M_PI);
                        else set_goal(cvs, 1.7775 + 0.1, .21, M_PI);
                    } else if (teensy->R_mes4 == 2) {
                        excSq->status = rec5_es;
                        printf("2 : go to rec5_es\n");
                        if (TEAM) set_goal(cvs, 1.71 - 0.01, 0.21, M_PI);
                        else set_goal(cvs, 1.4075 + 0.1, .21, M_PI);
                    } else if (teensy->R_mes4 == 0 || teensy->R_mes4 == 3) {
                        excSq->status = rec5_noR_es;
                        printf("0 : go to rec5_noR_es\n");
                        if (TEAM) set_goal(cvs, 1.71 - 0.01, 0.21, M_PI);
                        else set_goal(cvs, 1.4075 + 0.1, .21, M_PI);
                    }
                }
                break;
            }
            case rec5_es: {
                if (rec_static(cvs)) {
                    printf("rec5_es END : go to Dpmt5_prec_es\n");
                    excSq->status = Dpmt5_prec_es;
                }
                break;
            }
            case Dpmt5_prec_es: {
                //set_param_normal(cvs);
                set_param_prec(cvs);
                hlcPF->Tau_max = .15;
                hlcPF->Tau_min = .1;
                mlcPF->sigma = 0.5;
                sendFromHLCPF(cvs, -1, 1);
                if (hlcPF->output) {
                    motors_stop(cvs);
                    set_commands(cvs, 0, 0);
                    excSq->status = Check5_es;
                    setChrono(cvs, 2);
                    teensy_send(cvs, "T");
                    printf("go to Check5_es\n");
                }
                break;
            }
            case rec5_noR_es: {
                if (rec_static(cvs)) {
                    printf("rec5_noR_es END : go to Dpmt5_noR_es\n");
                    excSq->status = Dpmt5_noR_es;
                }
                break;
            }
            case Dpmt5_noR_es: {
                //set_param_normal(cvs);
                set_param_prec(cvs);
                hlcPF->Tau_max = .15;
                hlcPF->Tau_min = .1;
                mlcPF->sigma = 0.5;
                sendFromHLCPF(cvs, -1, 1);
                if (hlcPF->output) {
                    motors_stop(cvs);
                    set_commands(cvs, 0, 0);
                    excSq->status = Check5_noR_es;
                    setChrono(cvs, 2);
                    teensy_send(cvs, "C");
                    printf("go to Check5_noR_es\n");
                }
                break;
            }
            case Check5_noR_es: {
                if (teensy->R1) {
                    teensy->R_mes5 = 1;
                    if (excSq->at_least_one == 0) {
                        excSq->at_least_one = 1;
                        arduino_send(cvs,"A");
                        excSq->pts_flag[5] = 1;
                    } else {
                        if (excSq->pts_flag[5] == 0) {
                            arduino_send(cvs,"5");
                            excSq->pts_flag[5] = 1;
                        }
                    }
                } else if (teensy->R2) {
                    teensy->R_mes5 = 2;
                } else if (teensy->no_R) {
                    teensy->R_mes5 = 0;
                }
                if (checkChrono(cvs)) {
                    teensy->no_R = 0;
                    teensy->R1 = 0;
                    teensy->R2 = 0;
                    teensy->R3 = 0;
                    printf("1 OR 2 :\n");
                    if (teensy->R_mes5 == 1) {
                        excSq->status = rec6_es;
                        printf("1 : go to rec6_es\n");
                        if (TEAM) set_goal(cvs, 1.525 - 0.01, 0.21, M_PI);
                        else set_goal(cvs, 1.5925 + 0.1, .21, M_PI);
                    } else if (teensy->R_mes5 == 2) {
                        excSq->status = rec7_es;
                        printf("2 : go to rec7_es\n");
                        if (TEAM) set_goal(cvs, 1.34 - 0.01, 0.21, M_PI);
                        else set_goal(cvs, 1.7775 + 0.1, .21, M_PI);
                    } else if (teensy->R_mes5 == 0 || teensy->R_mes5 == 3) {
                        excSq->status = rec6_noR_es;
                        printf("0 : go to rec6_noR_es\n");
                        if (TEAM) set_goal(cvs, 1.525 - 0.01, 0.21, M_PI);
                        else set_goal(cvs, 1.5925 + 0.01, .21, M_PI);
                    }
                }
                break;
            }
            case Check5_es: {
                if (excSq->at_least_one == 0) {
                    excSq->at_least_one = 1;
                    arduino_send(cvs,"A");
                    excSq->pts_flag[5] = 1;
                } else {
                    if (excSq->pts_flag[5] == 0) {
                        arduino_send(cvs,"5");
                        excSq->pts_flag[5] = 1;
                    }
                }
                if (checkChrono(cvs)) {
                    excSq->status = rec6_es;
                    printf("go to rec6_es\n");
                    if (TEAM) set_goal(cvs, 1.525 - 0.01, 0.21, M_PI);
                    else set_goal(cvs, 3 - 1.525 + 0.01, .2, M_PI);
                }
                break;
            }
            case rec6_es: {
                if (rec_static(cvs)) {
                    printf("rec6_es END : go to Dpmt6_prec_es\n");
                    excSq->status = Dpmt6_prec_es;
                }
                break;
            }
            case Dpmt6_prec_es: {
                //set_param_normal(cvs);
                set_param_prec(cvs);
                hlcPF->Tau_max = .15;
                hlcPF->Tau_min = .1;
                mlcPF->sigma = 0.5;
                sendFromHLCPF(cvs, -1, 1);
                if (hlcPF->output) {
                    motors_stop(cvs);
                    set_commands(cvs, 0, 0);
                    excSq->status = Check6_es;
                    setChrono(cvs, 2);
                    teensy_send(cvs, "T");
                    printf("go to Check6_es\n");
                }
                break;
            }
            case rec6_noR_es: {
                if (rec_static(cvs)) {
                    printf("rec6_noR_es END : go to Dpmt6_noR_es\n");
                    excSq->status = Dpmt6_noR_es;
                }
                break;
            }
            case Dpmt6_noR_es: {
                //set_param_normal(cvs);
                set_param_prec(cvs);
                hlcPF->Tau_max = .15;
                hlcPF->Tau_min = .1;
                mlcPF->sigma = 0.5;
                sendFromHLCPF(cvs, -1, 1);
                if (hlcPF->output) {
                    motors_stop(cvs);
                    set_commands(cvs, 0, 0);
                    excSq->status = Check6_noR_es;
                    setChrono(cvs, 2);
                    teensy_send(cvs, "C");
                    printf("go to Check6_noR_es\n");
                }
                break;
            }
            case Check6_noR_es: {
                if (teensy->R1) {
                    teensy->R_mes6 = 1;
                    if (excSq->at_least_one == 0) {
                        excSq->at_least_one = 1;
                        arduino_send(cvs,"A");
                        excSq->pts_flag[6] = 1;
                    } else {
                        if (excSq->pts_flag[6] == 0) {
                            arduino_send(cvs,"5");
                            excSq->pts_flag[6] = 1;
                        }
                    }
                } else if (teensy->R2) {
                    teensy->R_mes6 = 2;
                } else if (teensy->no_R) {
                    teensy->R_mes6 = 0;
                }
                if (checkChrono(cvs)) {
                    teensy->no_R = 0;
                    teensy->R1 = 0;
                    teensy->R2 = 0;
                    teensy->R3 = 0;
                    if (teensy->R_mes6 == 1) {
                        excSq->status = Out_es;
                        printf("1 : go to Out_es\n");
                        if (TEAM) set_goal(cvs, 1.1, 0.3, -10);
                        else set_goal(cvs, 1.1, 0.3, -10);
                    } else if (teensy->R_mes6 == 2) {
                        excSq->status = rec7_es;
                        printf("2 : go to rec7_es\n");
                        if (TEAM) set_goal(cvs, 1.34 - 0.01, 0.21, M_PI);
                        else set_goal(cvs, 1.7775 + 0.01, .21, M_PI);
                    } else if (teensy->R_mes6 == 0 || teensy->R_mes6 == 3) {
                        excSq->status = rec7_noR_es;
                        printf("0 : go to rec7_noR_es\n");
                        if (TEAM) set_goal(cvs, 1.34 - 0.01, 0.21, M_PI);
                        else set_goal(cvs, 1.7775 + 0.01, .21, M_PI);
                    }
                }
                break;
            }
            case Check6_es: {
                if (excSq->at_least_one == 0) {
                    excSq->at_least_one = 1;
                    arduino_send(cvs,"A");
                    excSq->pts_flag[6] = 1;
                } else {
                    if (excSq->pts_flag[6] == 0) {
                        arduino_send(cvs,"5");
                        excSq->pts_flag[6] = 1;
                    }
                }
                if (checkChrono(cvs)) {
                    excSq->status = Out_es;
                    printf("go to Out_es\n");
                    if (TEAM) set_goal(cvs, 1.1, 0.3, -10);
                    else set_goal(cvs, 1.1, 0.3, -10);
                }
                break;
            }
            case rec7_es: {
                if (rec_static(cvs)) {
                    printf("rec7_es END : go to Dpmt7_prec_es\n");
                    excSq->status = Dpmt7_prec_es;
                }
                break;
            }
            case Dpmt7_prec_es: {
                //set_param_normal(cvs);
                set_param_prec(cvs);
                hlcPF->Tau_max = .15;
                hlcPF->Tau_min = .1;
                mlcPF->sigma = 0.5;
                sendFromHLCPF(cvs, -1, 1);
                if (hlcPF->output) {
                    motors_stop(cvs);
                    set_commands(cvs, 0, 0);
                    excSq->status = Check7_es;
                    setChrono(cvs, 2);
                    teensy_send(cvs, "T");
                    printf("go to Check7_es\n");
                }
                break;
            }
            case rec7_noR_es: {
                if (rec_static(cvs)) {
                    printf("rec7_noR_es END : go to Dpmt7_noR_es\n");
                    excSq->status = Dpmt7_noR_es;
                }
                break;
            }
            case Dpmt7_noR_es: {
                //set_param_normal(cvs);
                set_param_prec(cvs);
                hlcPF->Tau_max = .15;
                hlcPF->Tau_min = .1;
                mlcPF->sigma = 0.5;
                sendFromHLCPF(cvs, -1, 1);
                if (hlcPF->output) {
                    motors_stop(cvs);
                    set_commands(cvs, 0, 0);
                    excSq->status = Check7_es;
                    setChrono(cvs, 2);
                    teensy_send(cvs, "C");
                    printf("go to Check7_es\n");
                }
                break;
            }
            case Check7_es: {
                if (excSq->at_least_one == 0) {
                    excSq->at_least_one = 1;
                    arduino_send(cvs,"A");
                    excSq->pts_flag[7] = 1;
                } else {
                    if (excSq->pts_flag[7] == 0) {
                        arduino_send(cvs,"5");
                        excSq->pts_flag[7] = 1;
                    }
                }
                if (checkChrono(cvs)) {
                    excSq->status = Out_es;
                    printf("go to Out_es\n");
                    if (TEAM) set_goal(cvs, 1.1, 0.3, -10);
                    else set_goal(cvs, 1.1, 0.3, -10);
                }
                break;
            }
            case Out_es: {
                set_param_normal(cvs);
                sendFromHLCPF(cvs, 1,1);
                if (hlcPF->output) {
                    motors_stop(cvs);
                    set_commands(cvs, 0, 0);
                    printf("END\n");
                    excSq->output = 1;
                }
                break;
            
            default:
                printf("probleme defautl value in FSM\n");
                excSq->output = 1;
        }
    }
    else {
        switch(excSq->status){
            case S0_es:{
                if(excSq->go){
                    excSq->status = Dpmt1_es;
                    set_goal(cvs,1.965,0.48,M_PI/2);
                    printf("go to dp1\n");
                    excSq->go = 0;
                }
                break;
            }

            case Dpmt1_es:{
                sendFromHLCPF(cvs,-1);
                if(hlcPF->output){
                    motors_stop(cvs);
                    set_commands(cvs,0,0);
                    excSq->status = Dpmt2_es;
                    set_goal(cvs,1.935,0.2,M_PI);
                }
                break;
            }

            case Dpmt2_es:{
                sendFromHLCPF(cvs,-1,1);
                if(hlcPF->output){
                    motors_stop(cvs);
                    set_commands(cvs,0,0);
                    excSq->status = rec1_es;
                    set_goal(cvs,1.895-0.01,.21,M_PI);
                }
                break;
            }
            case rec1_es:{
                if (rec_static(cvs)) {
                    printf("rec1_es END : go to Dpmt1_prec_es\n");
                    excSq->status = Dpmt1_prec_es;
                }
                break;
            }
            case Dpmt1_prec_es:{
                //set_param_normal(cvs);
                set_param_prec(cvs);
                hlcPF->Tau_max = .15;
                hlcPF->Tau_min = .1;
                mlcPF->sigma = 0.5;
                sendFromHLCPF(cvs,-1,1);
                if(hlcPF->output){
                    motors_stop(cvs);
                    set_commands(cvs,0,0);
                    excSq->status = Check1_es;
                    setChrono(cvs,2);
                    teensy_send(cvs, "C");
                    printf("go to Check1_es\n");
                    //excSq->output = 1;
                }
                break;
            }
            case Check1_es:{
                if (teensy->R1) {
                    teensy->R_mes1 = 1;
                }
                else if (teensy->R2) {
                    teensy->R_mes1 = 2;
                    if (excSq->at_least_one == 0) {
                        excSq->at_least_one = 1;
                        arduino_send(cvs,"A");
                        excSq->pts_flag[1] = 1;
                    } else {
                        if (excSq->pts_flag[1] == 0) {
                            arduino_send(cvs,"5");
                            excSq->pts_flag[1] = 1;
                        }
                    }
                }
                if(checkChrono(cvs)){
                    teensy->no_R = 0;
                    teensy->R1 = 0;
                    teensy->R2 = 0;
                    teensy->R3 = 0;
                    if (teensy->R_mes1 == 1) {
                        excSq->status = rec2_es;
                        printf("1 : go to rec2_es\n");
                        set_goal(cvs,1.71-0.01,.21,M_PI);
                    }
                    else if (teensy->R_mes1 == 2) {
                        excSq->status = rec4_es;
                        printf("2 : go to rec4_es\n");
                        set_goal(cvs,1.34-0.01,.21,M_PI);
                    }
                    else if (teensy->R_mes1 == 0 || teensy->R_mes1 == 3) {
                        excSq->status = rec2_noR_es;
                        printf("0 : go to rec2_noR_es\n");
                        set_goal(cvs,1.71-0.01,.21,M_PI);
                    }
                }
                break;
            }
            case rec2_es:{
                if (rec_static(cvs)) {
                    printf("rec2_es END : go to Dpmt2_prec_es\n");
                    excSq->status = Dpmt2_prec_es;
                }
                break;
            }
            case Dpmt2_prec_es:{
                //set_param_normal(cvs);
                set_param_prec(cvs);
                hlcPF->Tau_max = .15;
                hlcPF->Tau_min = .1;
                mlcPF->sigma = 0.5;
                sendFromHLCPF(cvs,-1,1);
                if(hlcPF->output){
                    motors_stop(cvs);
                    set_commands(cvs,0,0);
                    excSq->status = Check2_es;
                    setChrono(cvs,2);
                    teensy_send(cvs, "T");
                    printf("go to Check2_es\n");
                }
                break;
            }
            case rec2_noR_es:{
                if (rec_static(cvs)) {
                    printf("rec2_noR_es END : go to Dpmt2_noR_es\n");
                    excSq->status = Dpmt2_noR_es;
                }
                break;
            }
            case Dpmt2_noR_es:{
                //set_param_normal(cvs);
                set_param_prec(cvs);
                hlcPF->Tau_max = .15;
                hlcPF->Tau_min = .1;
                mlcPF->sigma = 0.5;
                sendFromHLCPF(cvs,-1,1);
                if(hlcPF->output){
                    motors_stop(cvs);
                    set_commands(cvs,0,0);
                    excSq->status = Check2_noR_es;
                    setChrono(cvs,2);
                    teensy_send(cvs, "C");
                    printf("go to Check2_noR_es\n");
                }
                break;
            }
            case Check2_noR_es:{
                if (teensy->R1) {
                    teensy->R_mes2 = 1;
                }
                else if (teensy->R2) {
                    teensy->R_mes2 = 2;
                    if (excSq->at_least_one == 0) {
                        excSq->at_least_one = 1;
                        arduino_send(cvs,"A");
                        excSq->pts_flag[2] = 1;
                    } else {
                        if (excSq->pts_flag[2] == 0) {
                            arduino_send(cvs,"5");
                            excSq->pts_flag[2] = 1;
                        }
                    }
                }
                else if (teensy->no_R) {
                    teensy->R_mes2 = 0;
                }
                if(checkChrono(cvs)){
                    teensy->no_R = 0;
                    teensy->R1 = 0;
                    teensy->R2 = 0;
                    teensy->R3 = 0;
                    printf("1 OR 2 :\n");
                    if (teensy->R_mes2 == 2) {
                        excSq->status = rec3_es;
                        printf("1 : go to rec3_es\n");
                        set_goal(cvs,1.521-0.01,.21,M_PI);
                    }
                    else if (teensy->R_mes2 == 1) {
                        excSq->status = rec4_es;
                        printf("2 : go to rec4_es\n");
                        set_goal(cvs,1.34-0.01,.21,M_PI);
                    }
                    else if (teensy->R_mes2 == 0 || teensy->R_mes2 == 3) {
                        excSq->status = rec3_noR_es;
                        printf("0 : go to rec3_noR_es\n");
                        set_goal(cvs,1.521-0.01,.21,M_PI);
                    }
                }
                break;
            }
            case Check2_es:{
                if (excSq->at_least_one == 0) {
                    excSq->at_least_one = 1;
                    arduino_send(cvs,"A");
                    excSq->pts_flag[2] = 1;
                } else {
                    if (excSq->pts_flag[2] == 0) {
                        arduino_send(cvs,"5");
                        excSq->pts_flag[2] = 1;
                    }
                }
                if(checkChrono(cvs)){
                    excSq->status = rec3_es;
                    printf("go to rec3_es\n");
                    set_goal(cvs,1.521-0.01,0.21,M_PI);
                }
                break;
            }
            case rec3_es:{
                if (rec_static(cvs)) {
                    printf("rec3_es END : go to Dpmt3_prec_es\n");
                    excSq->status = Dpmt3_prec_es;
                }
                break;
            }
            case Dpmt3_prec_es:{
                //set_param_normal(cvs);
                set_param_prec(cvs);
                hlcPF->Tau_max = .15;
                hlcPF->Tau_min = .1;
                mlcPF->sigma = 0.5;
                sendFromHLCPF(cvs,-1,1);
                if(hlcPF->output){
                    motors_stop(cvs);
                    set_commands(cvs,0,0);
                    excSq->status = Check3_es;
                    setChrono(cvs,2);
                    teensy_send(cvs, "T");
                    printf("go to Check3_es\n");
                }
                break;
            }
            case rec3_noR_es:{
                if (rec_static(cvs)) {
                    printf("rec3_noR_es END : go to Dpmt3_noR_es\n");
                    excSq->status = Dpmt3_noR_es;
                }
                break;
            }
            case Dpmt3_noR_es:{
                //set_param_normal(cvs);
                set_param_prec(cvs);
                hlcPF->Tau_max = .15;
                hlcPF->Tau_min = .1;
                mlcPF->sigma = 0.5;
                sendFromHLCPF(cvs,-1,1);
                if(hlcPF->output){
                    motors_stop(cvs);
                    set_commands(cvs,0,0);
                    excSq->status = Check3_noR_es;
                    setChrono(cvs,2);
                    teensy_send(cvs, "C");
                    printf("go to Check3_noR_es\n");
                }
                break;
            }
            case Check3_noR_es:{
                if (teensy->R1) {
                    teensy->R_mes3 = 1;
                }
                else if (teensy->R2) {
                    teensy->R_mes3 = 2;
                    if (excSq->at_least_one == 0) {
                        excSq->at_least_one = 1;
                        arduino_send(cvs,"A");
                        excSq->pts_flag[3] = 1;
                    } else {
                        if (excSq->pts_flag[3] == 0) {
                            arduino_send(cvs,"5");
                            excSq->pts_flag[3] = 1;
                        }
                    }
                }
                else if (teensy->no_R) {
                    teensy->R_mes3 = 0;
                }
                if(checkChrono(cvs)){
                    teensy->no_R = 0;
                    teensy->R1 = 0;
                    teensy->R2 = 0;
                    teensy->R3 = 0;
                    printf("1 OR 2 :\n");
                    if (teensy->R_mes3 == 1) {
                        excSq->status = rec4_es;
                        printf("1 : go to rec4_es\n");
                        set_goal(cvs,1.34-0.01,.21,M_PI);
                    }
                    else if (teensy->R_mes3 == 2) {
                        excSq->status = rec5_es;
                        printf("2 : go to rec5_es\n");
                        set_goal(cvs,1.155-0.01,.21,M_PI);
                    }
                    else if (teensy->R_mes3 == 0 || teensy->R_mes2 == 3) {
                        excSq->status = rec4_noR_es;
                        printf("0 : go to rec4_noR_es\n");
                        set_goal(cvs,1.34-0.01,.21,M_PI);
                    }
                }
                break;
            }
            case Check3_es:{
                if (excSq->at_least_one == 0) {
                    excSq->at_least_one = 1;
                    arduino_send(cvs,"A");
                    excSq->pts_flag[3] = 1;
                } else {
                    if (excSq->pts_flag[3] == 0) {
                        arduino_send(cvs,"5");
                        excSq->pts_flag[3] = 1;
                    }
                }
                if(checkChrono(cvs)){
                    excSq->status = rec5_es;
                    printf("go to rec5_es\n");
                    set_goal(cvs,1.155-0.01,0.21,M_PI);
                }
                break;
            }
            case rec4_es:{
                if (rec_static(cvs)) {
                    printf("rec4_es END : go to Dpmt4_prec_es\n");
                    excSq->status = Dpmt4_prec_es;
                }
                break;
            }
            case Dpmt4_prec_es:{
                //set_param_normal(cvs);
                set_param_prec(cvs);
                hlcPF->Tau_max = .15;
                hlcPF->Tau_min = .1;
                mlcPF->sigma = 0.5;
                sendFromHLCPF(cvs,-1,1);
                if(hlcPF->output){
                    motors_stop(cvs);
                    set_commands(cvs,0,0);
                    excSq->status = Check4_es;
                    setChrono(cvs,2);
                    teensy_send(cvs, "T");
                    printf("go to Check4_es\n");
                }
                break;
            }
            case rec4_noR_es:{
                if (rec_static(cvs)) {
                    printf("rec4_noR_es END : go to Dpmt4_noR_es\n");
                    excSq->status = Dpmt4_noR_es;
                }
                break;
            }
            case Dpmt4_noR_es:{
                //set_param_normal(cvs);
                set_param_prec(cvs);
                hlcPF->Tau_max = .15;
                hlcPF->Tau_min = .1;
                mlcPF->sigma = 0.5;
                sendFromHLCPF(cvs,-1,1);
                if(hlcPF->output){
                    motors_stop(cvs);
                    set_commands(cvs,0,0);
                    excSq->status = Check4_es;
                    setChrono(cvs,2);
                    teensy_send(cvs, "C");
                    printf("go to Check4_es\n");
                }
                break;
            }
            case Check4_es:{
                if (excSq->at_least_one == 0) {
                    excSq->at_least_one = 1;
                    arduino_send(cvs,"A");
                    excSq->pts_flag[4] = 1;
                } else {
                    if (excSq->pts_flag[4] == 0) {
                        arduino_send(cvs,"5");
                        excSq->pts_flag[4] = 1;
                    }
                }
                if(checkChrono(cvs)){
                    excSq->status = rec5_es;
                    printf("go to rec5_es\n");
                    set_goal(cvs,1.34-0.01,0.21,M_PI);
                }
                break;
            }
            case rec5_es:{
                if (rec_static(cvs)) {
                    printf("rec5_es END : go to Dpmt5_prec_es\n");
                    excSq->status = Dpmt5_prec_es;
                }
                break;
            }
            case Dpmt5_prec_es:{
                //set_param_normal(cvs);
                set_param_prec(cvs);
                hlcPF->Tau_max = .15;
                hlcPF->Tau_min = .1;
                mlcPF->sigma = 0.5;
                sendFromHLCPF(cvs,-1,1);
                if(hlcPF->output){
                    motors_stop(cvs);
                    set_commands(cvs,0,0);
                    excSq->status = Check5_es;
                    setChrono(cvs,2);
                    teensy_send(cvs, "C");
                    printf("go to Check5_es\n");
                }
                break;
            }
            case Check5_es:{
                if (teensy->R1) {
                    teensy->R_mes5 = 1;
                }
                else if (teensy->R2) {
                    teensy->R_mes5 = 2;
                    if (excSq->at_least_one == 0) {
                        excSq->at_least_one = 1;
                        arduino_send(cvs,"A");
                        excSq->pts_flag[5] = 1;
                    } else {
                        if (excSq->pts_flag[5] == 0) {
                            arduino_send(cvs,"5");
                            excSq->pts_flag[5] = 1;
                        }
                    }
                }
                else if (teensy->no_R) {
                    teensy->R_mes5 = 0;
                }
                if(checkChrono(cvs)){
                    teensy->no_R = 0;
                    teensy->R1 = 0;
                    teensy->R2 = 0;
                    teensy->R3 = 0;
                    excSq->status = rec6_es;
                    printf("go to rec6_es\n");
                    set_goal(cvs,0.97-0.01,0.21,M_PI);
                }
                break;
            }
            case rec6_es:{
                if (rec_static(cvs)) {
                    printf("rec6_es END : go to Dpmt6_prec_es\n");
                    excSq->status = Dpmt6_prec_es;
                }
                break;
            }
            case Dpmt6_prec_es:{
                //set_param_normal(cvs);
                set_param_prec(cvs);
                hlcPF->Tau_max = .15;
                hlcPF->Tau_min = .1;
                mlcPF->sigma = 0.5;
                sendFromHLCPF(cvs,-1,1);
                if(hlcPF->output){
                    motors_stop(cvs);
                    set_commands(cvs,0,0);
                    excSq->status = Check6_es;
                    setChrono(cvs,2);
                    teensy_send(cvs, "T");
                    printf("go to Check6_es\n");
                }
                break;
            }
            case Check6_es:{
                if (excSq->at_least_one == 0) {
                    excSq->at_least_one = 1;
                    arduino_send(cvs,"A");
                    excSq->pts_flag[6] = 1;
                } else {
                    if (excSq->pts_flag[6] == 0) {
                        arduino_send(cvs,"5");
                        excSq->pts_flag[6] = 1;
                    }
                }
                if(checkChrono(cvs)){
                    teensy->no_R = 0;
                    teensy->R1 = 0;
                    teensy->R2 = 0;
                    teensy->R3 = 0;
                    printf("1 OR 2 :\n");
                    if (teensy->R_mes5 == 1) {
                        excSq->status = rec7_es;
                        printf("1 : go to rec7_es\n");
                        set_goal(cvs,0.785-0.01,.21,M_PI);
                    }
                    else if (teensy->R_mes5 == 2) {
                        excSq->status = Out_es;
                        printf("2 : go to Out_es\n");
                        set_goal(cvs,0.5,0.5,M_PI/2);
                    }
                    else if (teensy->R_mes5 == 0 || teensy->R_mes5 == 3) {
                        excSq->status = rec7_noR_es;
                        printf("0 : go to rec7_noR_es\n");
                        set_goal(cvs,0.785-0.01,.21,M_PI);
                    }
                }
                break;
            }
            case rec7_es:{
                if (rec_static(cvs)) {
                    printf("rec7_es END : go to Dpmt7_prec_es\n");
                    excSq->status = Dpmt7_prec_es;
                }
                break;
            }
            case Dpmt7_prec_es:{
                //set_param_normal(cvs);
                set_param_prec(cvs);
                hlcPF->Tau_max = .15;
                hlcPF->Tau_min = .1;
                mlcPF->sigma = 0.5;
                sendFromHLCPF(cvs,-1,1);
                if(hlcPF->output){
                    motors_stop(cvs);
                    set_commands(cvs,0,0);
                    excSq->status = Check7_es;
                    setChrono(cvs,2);
                    teensy_send(cvs, "T");
                    printf("go to Check7_es\n");
                }
                break;
            }
            case rec7_noR_es:{
                if (rec_static(cvs)) {
                    printf("rec7_noR_es END : go to Dpmt7_noR_es\n");
                    excSq->status = Dpmt7_noR_es;
                }
                break;
            }
            case Dpmt7_noR_es:{
                //set_param_normal(cvs);
                set_param_prec(cvs);
                hlcPF->Tau_max = .15;
                hlcPF->Tau_min = .1;
                mlcPF->sigma = 0.5;
                sendFromHLCPF(cvs,-1,1);
                if(hlcPF->output){
                    motors_stop(cvs);
                    set_commands(cvs,0,0);
                    excSq->status = Check7_es;
                    setChrono(cvs,2);
                    teensy_send(cvs, "C");
                    printf("go to Check7_es\n");
                }
                break;
            }
            case Check7_es:{
                if (excSq->at_least_one == 0) {
                    excSq->at_least_one = 1;
                    arduino_send(cvs,"A");
                    excSq->pts_flag[7] = 1;
                } else {
                    if (excSq->pts_flag[7] == 0) {
                        arduino_send(cvs,"5");
                        excSq->pts_flag[7] = 1;
                    }
                }
                if(checkChrono(cvs)){
                    excSq->status = Out_es;
                    printf("go to Out_es\n");
                    set_goal(cvs,0.5,0.5,M_PI/2);
                }
                break;
            }
            case Out_es:{
                set_param_normal(cvs);
                sendFromHLCPF(cvs,1);
                if(hlcPF->output){
                    motors_stop(cvs);
                    set_commands(cvs,0,0);
                    printf("END\n");
                    excSq->output = 1;
                }
                break;
            }
            default:
                printf("probleme defautl value in FSM\n");
                excSq->output = 1;
        }
    }

}