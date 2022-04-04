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
    //outputs
    hlcPF->output_main = 0;
    hlcPF->output = 0;
    hlcPF->d = 0;
    //param physique
    hlcPF->x_shift = 0.0625;
    //hlcPF->x_shift = 0.035;
    hlcPF->error = 0.075;//0.03;
    //references
    hlcPF->v_ref = 0.0;
    hlcPF->theta_ref = 0.0;
    hlcPF->vx = 0.0;
    hlcPF->vy = 0.0;
    //Forces
    hlcPF->F_att[0] = 0; hlcPF->F_att[1] = 0;
    hlcPF->F_rep[0] = 0; hlcPF->F_rep[1] = 0;
    //parameters
    hlcPF->maxF_att = .98;
    hlcPF->maxF_rep = 1;
    hlcPF->goal[0] = 0;hlcPF->goal[1] = 0;
    //repulsive static
    hlcPF->Eta = 0.02; //.025 = tout tout juste !!
    hlcPF->Rho = 0.4; // 
    //param dyn obstacle
    hlcPF->Eta_opp = 0.025*5;
    hlcPF->Rho_opp = 0.4*5;
    //hlcPF->Eta_opp = 0.03;
    //hlcPF->Rho_opp = 0.4;
    //attractive
    double a = 1;
    hlcPF->d_limit = 0.5;
    hlcPF->Alpha = a/hlcPF->d_limit;
    //tau
    hlcPF->Tau_max = 5;
    hlcPF->tau_max_dist = .8;
    hlcPF->Tau_min = 1;
    hlcPF->tau_min_dist = .2;
    //reorientation
    hlcPF->erreurTh = 0.1;
    //local minimum
    hlcPF->Fmin = 0.0000002;//0.03;
    hlcPF->flag_min_local = 0;
    hlcPF->begin_min_local_dodge = 0;
    hlcPF->goal_local_dodge[0] = 0; hlcPF->goal_local_dodge[1] = 0;
    hlcPF->dodge_incr = 0;
}


int get_partition_map(CtrlStruct *cvs){
    highLevelCtrlPF *hlcPF = cvs->hlcPF;
    myPosition *mp = cvs->mp;
    double th = mp->th;
    double x = mp->x + hlcPF->x_shift * cos(th);
    double y = mp->y + hlcPF->x_shift * sin(th);
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


void compute_incr(CtrlStruct *cvs, double th) {
    int part = get_partition_map(cvs);
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
    cvs->hlcPF->dodge_incr = toincr;
}

//make computation to give a tau dependant of the distance to the opponent (ralentit si trop proche de l'ennemi)

double tau_compute(CtrlStruct *cvs) {
    double tau_max = cvs->hlcPF->Tau_max;
    double tau_min = cvs->hlcPF->Tau_min;
    double tau_max_dist = cvs->hlcPF->tau_max_dist;
    double tau_min_dist = cvs->hlcPF->tau_min_dist;

    double a = (tau_min-tau_max)/(tau_min_dist-tau_max_dist);
    double b = tau_min - a*tau_min_dist;

    double x_opp = cvs->obs->obs_dyn_x[0];
    double y_opp = cvs->obs->obs_dyn_y[0];

    double d_opp = sqrt(x_opp*x_opp+y_opp*y_opp);
    double d = cvs->hlcPF->d;

    double tau_return = 0;
    double d_return = std::min(d,d_opp);

    tau_return = a*d_return+b;

    if (tau_return <= tau_min) tau_return = tau_min;
    else if (tau_return >= tau_max) tau_return = tau_max;

    set_plot(tau_return,"tau");
    //distance to opponent
    return tau_return;
}

void calc_AttractivePotential(CtrlStruct *cvs,double x_goal,double y_goal) {
    highLevelCtrlPF *hlcPF = cvs->hlcPF;
    myPosition *mp = cvs->mp;
    double ALPHA   = hlcPF->Alpha ;
    double d_limit = hlcPF->d_limit  ;

    double th = mp->th;
    double x = mp->x + hlcPF->x_shift * cos(th);
    double y = mp->y + hlcPF->x_shift * sin(th);

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
void calc_RepulsivePotential(CtrlStruct *cvs) {
    highLevelCtrlPF *hlcPF = cvs->hlcPF;
    obstacles *obs = cvs->obs;
    myPosition *mp = cvs->mp;

    double th = mp->th;
    double x = mp->x + hlcPF->x_shift * cos(th);
    double y = mp->y + hlcPF->x_shift * sin(th);

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
    double *x_dyn = obs->obs_dyn_x;
    double *y_dyn = obs->obs_dyn_y;
    NumbOfObst = obs->size_dyn;
    for (int i = 0;i<NumbOfObst;i++){

        double x1 = x - x_dyn[i];
        double y1 = y - y_dyn[i];
        ETHA = hlcPF->Eta_opp;
        front_obst = hlcPF->Rho_opp;

        double d = sqrt(x1*x1+y1*y1);
        if (d <= R){
            if (d <= front_obst) {
                if (abs(x-x_dyn[i]) <= 0.00001){
                    Fx += 100;
                }
                else{
                    Fx += ETHA * (1/front_obst - 1/d) * 1/(d*d) * (x-x_dyn[i])/2;
                }
                if (abs(y-y_dyn[i]) <= 0.00001){
                    Fy += 100;
                }
                else {
                    Fy += ETHA * (1/front_obst - 1/d) * 1/(d*d) * (y-y_dyn[i])/2;
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
}

void main_pot_force(CtrlStruct *cvs,double x_goal,double y_goal,int goForward,double orientation){
    highLevelCtrlPF *hlcPF = cvs->hlcPF;
    CtrlIn *inputs = cvs->inputs;
    //reset output
    hlcPF->output_main = 0;
    myPosition *mp = cvs->mp;

    double goalToSendX = x_goal;
    double goalToSendY = y_goal;

    if(hlcPF->flag_min_local){
        goalToSendX = hlcPF->goal_local_dodge[0];
        goalToSendY = hlcPF->goal_local_dodge[1];
        //printf("well in local min\n");
    }
    double th = mp->th;
    double x = mp->x + hlcPF->x_shift * cos(th);
    double y = mp->y + hlcPF->x_shift * sin(th);
    double minF = hlcPF->Fmin;
    double error = hlcPF->error;

    calc_AttractivePotential(cvs,goalToSendX,goalToSendY);
    calc_RepulsivePotential(cvs);

    double F_att_x = hlcPF->F_att[0];
    double F_att_y = hlcPF->F_att[1];
    double F_att = sqrt(F_att_x*F_att_x+F_att_y*F_att_y);
    double F_rep_x = hlcPF->F_rep[0];
    double F_rep_y = hlcPF->F_rep[1];
    double F_rep = sqrt(F_rep_x*F_rep_x+F_rep_y*F_rep_y);

    // si norme > 1 -> scale par rapport a la norme
    double maxF_att = hlcPF->maxF_att;
    double maxF_rep = hlcPF->maxF_rep;
    if ( F_att > maxF_att) {
        F_att_x /= F_att;
        F_att_x *= maxF_att;
        F_att_y /= F_att;
        F_att_y *= maxF_att;
    }
    if ( F_rep > maxF_rep) {
        F_rep_x /= F_rep;
        F_rep_x *= maxF_rep;
        F_rep_y /= F_rep;
        F_rep_y *= maxF_rep;
    } 

    double F_tot_x = (F_att_x + F_rep_x) * -1;
    double F_tot_y = (F_att_y + F_rep_y) * -1;
    double F_tot = sqrt(F_tot_x*F_tot_x+F_tot_y*F_tot_y);

    if (F_tot > 1) {
        F_tot_x /= F_tot;
        F_tot_y /= F_tot;
    }

    double d = sqrt((x-x_goal)*(x-x_goal) +(y-y_goal)*(y-y_goal));
    hlcPF->d  = d;
    double tau = tau_compute(cvs);

    double v_x = tau * F_tot_x;
    double v_y = tau * F_tot_y;
    double v = sqrt(v_x*v_x+v_y*v_y);
    double theta = atan(F_tot_y/F_tot_x);

    //to modify, check orientation because atan is -pi/2,pi/2
    //double d = sqrt((x-x_goal)*(x-x_goal) +(y-y_goal)*(y-y_goal));

    if (v_x < 0) theta = limit_angle(theta + M_PI);
    if(goForward == -1) {
        if ( abs(limit_angle(mp->th-theta)) > abs(limit_angle((mp->th+M_PI-theta))) ) {
            v = -v;
            theta = limit_angle(theta + M_PI);
        }
    }
    //to go backward
    if(goForward == 0){
        v = -v;
        theta = limit_angle(theta + M_PI);
        //printf("go backward\n");
    }

    //check for minima
    //check with new F's normalized
    F_att = sqrt(F_att_x*F_att_x+F_att_y*F_att_y);
    F_tot = sqrt(F_tot_x*F_tot_x+F_tot_y*F_tot_y);
    set_plot(F_att,"F_att");
    set_plot(F_tot,"F_tot");
    set_plot(d,"d hlcPF");
    //check if in a special range (is arrived)
    if (d<error) {
        double dth = limit_angle(orientation-cvs->mp->th);
        if(orientation==-10){
            hlcPF->output = 1;
            v = 0; theta = 0;
            hlcPF->output = 1;
            printf("goal :: x : %f | y: %f\n", x_goal,y_goal);
            printf("No re-orientation\n");
        } else if(abs(dth) > hlcPF->erreurTh){
           theta = limit_angle(orientation);
            printf("in reorientation\n");
        } else {
            hlcPF->output = 1;
            v = 0; theta = 0;
            hlcPF->output = 1;
            printf("goal :: x : %f | y: %f\n", x_goal,y_goal);
            printf("reoriented\n");
        }
    } 
    else if(hlcPF->flag_min_local) {
        if(inputs->t - hlcPF->begin_min_local_dodge > 1 ) {
            hlcPF->flag_min_local = 0;
            printf("reset local min\n");
        }
    } //check for minima 
    else if (F_att > minF and F_tot < minF){
        printf("Local minimum /: \n");
        //Angle
        double diffx = x_goal - x;
        double diffy = y_goal - y;
        double theta_g = theat_g_compute(diffx,diffy);
        printf("theta_g before compute %f\n",theta_g);
        compute_incr(cvs,theta_g);
        theta_g += hlcPF->dodge_incr;
        if(theta_g>3.1415) {
            theta_g -= 2*3.1415;
        } else if (theta_g< -3.1415) {
            theta_g += 2*3.1415;
        }
        printf("theta_g = %f\n",theta_g );
        //1 m from points goal
        hlcPF->goal_local_dodge[0] = x + hlcPF->x_shift*cos(th) + 1 * cos(theta_g);
        hlcPF->goal_local_dodge[1] = y + hlcPF->x_shift*sin(th) + 1 * sin(theta_g);
        hlcPF->begin_min_local_dodge = inputs->t;
        hlcPF->vx = 0;
        hlcPF->vy = 0;
        //set to 0 since minima 
        // -> have to recalculate with the new goal given to go out of the minimum
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
}
