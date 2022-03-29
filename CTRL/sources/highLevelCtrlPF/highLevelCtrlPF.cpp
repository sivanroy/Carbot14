/*!
 * \file highLevelCtrlPF_gr5.cc
 * \brief
 */
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <cmath> //for sqrt

#include "highLevelCtrlPF.h" // adapt it with your headers

void hlcPF_init(highLevelCtrlPF *hlcPF) {
    hlcPF->output = 0;
    hlcPF->d = 0;

    hlcPF->v_ref = 0.0;
    hlcPF->theta_ref = 0.0;

    hlcPF->vx = 0.0;
    hlcPF->vy = 0.0;

    hlcPF->F_att[0] = 0; hlcPF->F_att[1] = 0;
    hlcPF->F_rep[0] = 0; hlcPF->F_rep[1] = 0;

    hlcPF->Eta = 0.025; //.025 = tout tout juste !!
    hlcPF->Rho = 0.4; // 128 rayon pour le robot a l'avant

    double a = 2;
    hlcPF->d_limit = 0.2;
    hlcPF->Alpha = a/hlcPF->d_limit;

    hlcPF->Tau = 1.5;

    hlcPF->goal[0] = 0;hlcPF->goal[1] = 0;

    hlcPF->flag_min_local = 0;
    hlcPF->begin_min_local_dodge = 0;
    hlcPF->goal_local_dodge[0] = 0; hlcPF->goal_local_dodge[1] = 0;
    hlcPF->dodge_incr = 0;
}

int get_partition_map(ctrlStruct *cvs){
    highLevelCtrlPF *hlcPF = cvs->hlcPF;
    myPosition *mp = cvs->mp;

    double x = mp->x; double y = mp->y;
    if(x< 1) {
        if(x>y) {
            return 3;
        } else if (2-y < x) {
            return 1;
        } else {
            return 4;
        }
    } else if (x> 2){
        if(-x+3>y) {
            return 3;
        } else if(x-1<y){
            return 1;
        } else {
            return 2;
        }
    }
    else if (y>1){
        return 1;
    } else {
        return 3;
    } 
    return -1;

}

double theat_g_compute(double diffx,double diffy){
    double theta_g;
    if(diffx != 0) {
        double theta_g = atan(diffy/diffx);
        printf("theta_g computed here : %f\n",theta_g );
        if(diffx < 0) {
            theta_g += 3.1415;
        } 
        if(theta_g > 3.1415) {
            theta_g -= 2*3.1415;
        } else if (theta_g< -3.1415) {
            theta_g += 2*3.1415;
        }
        printf("finally computed here theta_g %f\n",theta_g );
        return theta_g;
    } else {
        printf("else !\n");
        if (diffy > 0) {
            theta_g = 3.1415/2;
            return theta_g;
        } else {
            theta_g = -3.1415/2;
            return theta_g;
        }
    } 
}


void calc_AttractivePotential(ctrlStruct *cvs,double x_goal,double y_goal) {
    highLevelCtrlPF *hlcPF = cvs->hlcPF;
    myPosition *mp = cvs->mp;
    double ALPHA   = hlcPF->Alpha ;
    double d_limit = hlcPF->d_limit  ;

    double x = mp->x; double y = mp->y;
    //double x_goal = hlcPF->goal[0]; double y_goal = hlcPF->goal[1];

    double x1 = x - x_goal;
    double y1 = y - y_goal;
    double d = sqrt(x1*x1+y1*y1);

    double Fx;
    double Fy;

    if (d <= d_limit) {
        Fx = ALPHA*(x - x_goal);
        Fy = ALPHA*(y - y_goal);
    } 
    else {
        Fx = (d_limit * ALPHA * (x - x_goal))/(d);
        Fy = (d_limit * ALPHA * (y - y_goal))/(d);
    }
    hlcPF->F_att[0] = Fx;
    hlcPF->F_att[1] = Fy;
}

//maximum 1 !!
void calc_RepulsivePotential(ctrlStruct *cvs) {
    highLevelCtrlPF *hlcPF = cvs->hlcPF;
    obstacles *obs = cvs->obs;
    myPosition *mp = cvs->mp;

    double x = mp->x; double y = mp->y;
    double ETHA =  hlcPF->Eta;     //# Scaling factor repulsive potential
    double front_obst = hlcPF->Rho; //# Influence dimension of obstacle

    double Fx = 0.0; double Fy = 0.0;
    double R = 1; //quid ??

    //obstacles statiques
    double *x_obst = obs->obs_stat_x;
    double *y_obst = obs->obs_stat_y;
    double NumbOfObst = obs->size_stat;
    for (int i = 0;i<NumbOfObst;i++){
        double x1 = x - x_obst[i];
        double y1 = y - y_obst[i];
        double d = sqrt(x1*x1+y1*y1);
        if (d <= R){
            if (d <= front_obst) {
                if (abs(x-x_obst[i]) <= 0.000001) {
                    Fx += 1;
                }
                else{
                    Fx += ETHA * (1/front_obst - 1/d) * 1/(d*d) * (x-x_obst[i])/2;
                }
                if (abs(y-y_obst[i]) <= 0.000001) {
                    Fy += 1;
                }
                else {
                    Fy += ETHA * (1/front_obst - 1/d) * 1/(d*d) * (y-y_obst[i])/2;
                }
            }
            else {
                Fx += 0 ;
                Fy += 0 ;
            }
        }
    }
    //obstacles dynamiques
    double x_dyn = obs->obs_dyn_x;
    double y_dyn = obs->obs_dyn_y;
    NumbOfObst = obs->size_dyn;
    for (int i = 0;i<NumbOfObst;i++){
        double x1 = x - x_dyn;
        double y1 = y - y_dyn;
        ETHA *= 5;
        front_obst *= 4;

        double d = sqrt(x1*x1+y1*y1);
        if (d <= R){
            if (d <= front_obst) {
                if (abs(x-x_dyn) <= 0.00001){
                    Fx += 100;
                }
                else{
                    Fx += ETHA * (1/front_obst - 1/d) * 1/(d*d) * (x-x_dyn)/2;
                }
                if (abs(y-y_dyn) <= 0.00001){
                    Fy += 100;
                }
                else {
                    Fy += ETHA * (1/front_obst - 1/d) * 1/(d*d) * (y-y_dyn)/2;
                }
            }
            else {
                Fx += 0 ;
                Fy += 0 ;
            }
        }
    } 
    hlcPF->F_rep[0] = Fx; 
    hlcPF->F_rep[1] = Fy;
    //printf("Frep_x : %f| Frep_y : %f\n",Fx,Fy);
}

void main_pot_force(ctrlStruct *cvs,double x_goal,double y_goal){
    highLevelCtrlPF *hlcPF = cvs->hlcPF;
    ctrlIn *inputs = cvs->inputs;
        //reset output
    hlcPF->output = 0;
    myPosition *mp = cvs->mp;


    double goalToSendX = x_goal;
    double goalToSendY = y_goal;

    if(hlcPF->flag_min_local){
        goalToSendX = hlcPF->goal_local_dodge[0];
        goalToSendY = hlcPF->goal_local_dodge[1];
        //printf("well in local min\n");
    }
    
    double x = mp->x;
    double y = mp->y;
    double theta_robot = mp->th;

    calc_AttractivePotential(cvs,goalToSendX,goalToSendY);
    calc_RepulsivePotential(cvs);

    double F_att_x = hlcPF->F_att[0];
    double F_att_y = hlcPF->F_att[1];
    double F_att = sqrt(F_att_x*F_att_x+F_att_y*F_att_y);
    double F_rep_x = hlcPF->F_rep[0];
    double F_rep_y = hlcPF->F_rep[1];
    double F_rep = sqrt(F_rep_x*F_rep_x+F_rep_y*F_rep_y);


    // si norme > 1 -> scale par rapport a la norme
    if ( F_att > .98) {
        F_att_x /= F_att;
        F_att_x *= .98;
        F_att_y /= F_att;
        F_att_y *= .98;
    }
    if ( F_rep > 1) {
        F_rep_x /= F_rep;
        F_rep_y /= F_rep;
    } 

    double F_tot_x = (F_att_x + F_rep_x) * -1;
    double F_tot_y = (F_att_y + F_rep_y) * -1;
    double F_tot = sqrt(F_tot_x*F_tot_x+F_tot_y*F_tot_y);

    if (F_tot > 1) {
        F_tot_x /= F_tot;
        F_tot_y /= F_tot;
    }

    double tau = hlcPF->Tau;

    double v_x = tau * F_tot_x;
    double v_y = tau * F_tot_y;
    double v = sqrt(v_x*v_x+v_y*v_y);
    double theta = atan(F_tot_y/F_tot_x);

    //to modify, check orientation because atan is -pi/2,pi/2
    double d = sqrt((x-x_goal)*(x-x_goal) +(y-y_goal)*(y-y_goal));
    if (v_x < 0) {
        theta += 3.1415;
        if (theta > 3.1415) {
            theta -= 2*3.1415;
        }
    }

    //check for minima
    //check with new F's normalized
    F_att = sqrt(F_att_x*F_att_x+F_att_y*F_att_y);
    F_tot = sqrt(F_tot_x*F_tot_x+F_tot_y*F_tot_y);

    double minF = 0.03;
    //printf("fatt %f | ftot %f \n",F_att,F_tot );
    double error = 0.03;

    //check if in a special range (is arrived)
    if (d<error) {
            v = 0;
            hlcPF->output = 1;
            printf("goal :: x : %f | y: %f\n", x_goal,y_goal);
    } 
    else if(hlcPF->flag_min_local) {
        if(inputs->t - hlcPF->begin_min_local_dodge > 1 ) {
            hlcPF->flag_min_local = 0;
            printf("reset local min\n");
        }
    } //check for minima 


    else if (F_att > minF and F_tot < minF){//} | hlcPF->flag_min_local) {
        printf("Local minimum /: \n");
        //Angle
        double diffx = x_goal - x;
        double diffy = y_goal - y;

        double theta_g = theat_g_compute(diffx,diffy);

        printf("theta_g before compute %f\n",theta_g);

        int part = get_partition_map(cvs);
        printf("part %d\n",part );
        double th = theta_g;//cvs->mp->th;

        double incr = 3.1415/2;
        double toincr = 0;
        if(part == 1) {
            if(th>3.1415/2 | th< -3.1415/2){
                toincr += incr;
            } else {
                toincr -= incr;
            }
        } else if (part == 2) {
            if(th> 0) {
                toincr+= incr;
            } else {
                toincr-=incr;
            }
        } else if (part == 3) {
            if(th>3.1415/2 | th< -3.1415/2){
                toincr -= incr;
            } else {
                toincr += incr;
            }
        } else if (part == 4){
            if(th> 0) {
                toincr -= incr;
            } else {
                toincr += incr;
            }
        } else {
            printf("should not happend !! part\n");
        }

        hlcPF->dodge_incr = toincr;
        printf("incr %f\n",toincr); 

        theta_g += toincr;

        if(theta_g>3.1415) {
            theta_g -= 2*3.1415;
        } else if (theta_g< -3.1415) {
            theta_g += 2*3.1415;
        }

        printf("theta_g = %f\n",theta_g );

        //1 m from points goal
        hlcPF->goal_local_dodge[0] = x + .3 * cos(theta_g);
        hlcPF->goal_local_dodge[1] = y + .3 * sin(theta_g);
        hlcPF->begin_min_local_dodge = inputs->t;
        hlcPF->vx = 0;
        hlcPF->vy = 0;

        hlcPF->v_ref = 0;
        hlcPF->theta_ref = 0;

        hlcPF->flag_min_local = 1;
        return;    
    }

    hlcPF->vx = v_x;
    hlcPF->vy = v_y;

    hlcPF->v_ref = v;
    hlcPF->theta_ref = theta;
    hlcPF->d = d;

    //int part = get_partition_map(cvs);
    //printf("part %d\n",part );
    //printf("v: %f | theta %f\n",v,theta );
}

