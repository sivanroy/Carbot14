#include "pushShed.h"


#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <cmath> //for sqrt

#include "pushShed.h"



//enum enum {S0_ps,Dpmt1_ps,Dpmt2_ps,Close_ps,Dpmt3_ps,Dpmt4_ps,Push_ps,Ok_ps,NotOk_ps};

void pushShed_init(pushShed *pshed) {
    pshed->status = S0_ps;
    pshed->output = 0;
    pshed->go = 1;
    int s = 4;
    double x_goalsI[s] = {1,1,1,1};
    double y_goalsI[s] = {1,1,1,1};
    for (int i=0; i<s;i++) {
    	pshed->x_goals[i] = x_goalsI[i];
    	pshed->y_goals[i] = y_goalsI[i];
    }
}

void pushShed_launch(ctrlStruct *cvs){
	cvs->pshed->go = 1;
	cvs->pshed->status = S0_ps;
}

void sendFromMainPot(ctrlStruct *cvs,double x_goal,double y_goal){
	get_d2r_data(cvs); 
    main_pot_force(cvs,x_goal,y_goal);
    mlcPF_out(cvs, cvs->hlcPF->v_ref, cvs->hlcPF->theta_ref);
    printf("hlcPF->v %f | hlcPF->theta %f\n",cvs->hlcPF->v_ref,cvs->hlcPF->theta_ref );
    set_commands(cvs, cvs->mlcPF->r_sp_ref, cvs->mlcPF->l_sp_ref);
    send_commands(cvs);
    set_new_position(cvs);
}

void pushShed_loop(ctrlStruct *cvs){
	pushShed *pshed = cvs->pshed;
    myPosition *mp = cvs->mp;
    ctrlIn  *inputs = cvs->inputs;
    highLevelCtrlPF *hlcPF;

    double x = mp->x; double y = mp->y;
    double xg; double yg;

    switch(pshed->status){
        case S0_ps:
        	if(pshed->go){
        		pshed->status = Dpmt1_ps;
        	}
            break;

        case Dpmt1_ps:{
            xg = pshed->x_goals[0];
    		yg = pshed->y_goals[0];
    		sendFromMainPot(cvs,xg,yg);
        	if(hlcPF->output){
        		pshed->status = Dpmt2_ps;
        	}
        	break;
        }

        case Dpmt2_ps:{
            pshed->status = Dpmt3_ps;
    		xg = pshed->x_goals[1];
    		yg = pshed->y_goals[1];
    		sendFromMainPot(cvs,xg,yg);
        	if(hlcPF->output){
        		pshed->status = Close_ps;
        	}
        	break;
        }

        case Close_ps:{
        	printf("close pushShed not implemented yet\n");
        	pshed->status = Dpmt3_ps;
        	break;
        }

        case Dpmt3_ps:{
            xg = pshed->x_goals[2];
        	yg = pshed->y_goals[2];
        	sendFromMainPot(cvs,xg,yg);
        	if(hlcPF->output){
        		pshed->status = Dpmt4_ps;
        	}
        	break;
        }

        case Dpmt4_ps:{
            xg = pshed->x_goals[3];
        	yg = pshed->y_goals[3];
        	sendFromMainPot(cvs,xg,yg);
        	if(hlcPF->output){
        		pshed->status = S0_ps;
        		printf("No feedback given yet into pushSHed\n");
        	}
        	break; 
        }   	

        default:
            printf("probleme defautl value in FSM\n");
    }

}

