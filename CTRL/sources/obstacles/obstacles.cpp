/*!
 * \file obstacles_gr5.cc
 * \brief File description
 */

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <math.h>
#include <cmath> //for sqrt

#include "obstacles.h"


void obs_init(obstacles *obs)
{
	obs->sizemax = 1000;
	obs->obs_dyn_x[0] = -10;
	obs->obs_dyn_y[0] = -10;
	obs->obs_stat_x[0] = -10;
	obs->obs_stat_y[0] = -10;
	obs->size_dyn = 0;
	obs->size_stat = 0;
	obs_stat(obs);
}

//initialisation of static obstacles
void obs_stat(obstacles *obs){ 
	int s = 14;
	//define here all the static obstacles !
    double limitsx[s][2] = {{0.56,2.45},{3.00,3.0}, {3.00,2.555},{.44,0},{1.175,1.825}  ,{0,0},
               {.51,0},{2.49,3.0},
               {1.45,1.5},{1.55,1.5},
               {2.898,2.898},{.102,.102},
               {.45,1.17},{1.830,2.55}};
    double limitsy[s][2] = {{0,0},  {0.57,2.00},   {2.00,2.00},{2.00,2.00},{2.00,2.00},  {2.00,0.57},
               {0,.51},{0,.51},
               {1.95,1.65},{1.95,1.65},//{1.95,1.7},
               {.825,.675},{.825,.675},
               {1.914,1.914},{1.914,1.914}}; 
    //double N[s] = {50,50,50,50,        10,10,       10,10,        10,10,        15,15,5,5,5,5};
    double dx = 0.05;
    double xy1[2];
    double xy2[2];
    for (int i = 0; i<s; i++){
        double *ys = limitsy[i];
        double *xs = limitsx[i];
        //int ni = N[i];
        xy1[0] = xs[0]; xy1[1] = ys[0];
        xy2[0] = xs[1]; xy2[1] = ys[1];
        lines(obs,xy1,xy2,dx,0);
    }
    printf("obstacles initialized\n");
}


void lines(obstacles *obs,double xy1[2],double xy2[2],double dx,int dyn){
    double d = sqrt(pow((xy1[0]-xy2[0]),2)+pow((xy1[1]-xy2[1]),2));
    int N = fmax( floor(d/dx) ,3 );
    if ( (xy2[0] - xy1[0]) == 0){
        for (int i = 0; i<N;i++){
            double y1 = xy1[1]; double y2 = xy2[1];
            double dy = y2-y1;
            double step = dy/(N-1);
            double newy = y1+step*i;
            double newx = xy1[0];
            if(dyn){
            	obs->obs_dyn_x[obs->size_dyn] = newx;
	            obs->obs_dyn_y[obs->size_dyn] = newy;
	            obs->size_dyn ++;  
        	}
        	else {
      			obs->obs_stat_x[obs->size_stat] = newx;
	            obs->obs_stat_y[obs->size_stat] = newy;
	            obs->size_stat ++;
        	}
        }
    }    
    else {
        double a = (xy2[1]-xy1[1]) / (xy2[0] - xy1[0]);
        double b = xy1[1] - a*xy1[0];
        double x1 = xy1[0]; double x2 = xy2[0];
        for (int i =0; i<N; i++){
            double dx = x2-x1;
            double step = dx / (N-1);
            double newx = x1+step*i;
            double newy = a*(x1+step*i)+b;
            if(dyn){
            	obs->obs_dyn_x[obs->size_dyn] = newx;
	            obs->obs_dyn_y[obs->size_dyn] = newy;
	            obs->size_dyn ++;  
                if (obs->size_dyn >= obs->sizemax) {
                    printf("Error with obstacles formation; to much points to fit int the 1000 arraty; actual size %d\n",obs->size_dyn );
                }
            }
        	else {
      			obs->obs_stat_x[obs->size_stat] = newx;
	            obs->obs_stat_y[obs->size_stat] = newy;
	            obs->size_stat ++;
                if (obs->size_stat >= obs->sizemax) {
                    printf("Error with obstacles formation; to much points to fit int the 1000 arraty; actual size %d\n",obs->size_stat );
                }
        	}
        }
    }
}


void dyn_obs_free(ctrlStruct *cvs){
    obstacles *obs = cvs->obs;
    obs->size_dyn = 0;
    obs->obs_dyn_x[0] = -10;
    obs->obs_dyn_y[0] = -10;
}


void dyn_obs_set(ctrlStruct *cvs,double *x,double *y,int size) {
    obstacles *obs=cvs->obs;
    for (int i =0; i<size;i++) {
        obs->obs_dyn_x[obs->size_dyn] = x[i];
        obs->obs_dyn_y[obs->size_dyn] = y[i];
        obs->size_dyn++;
        if (obs->size_dyn >= obs->sizemax) {
            printf("Error with obstacles formation; to much points to fit int the 1000 arraty; actual size %d\n",obs->size_dyn );
            obs->size_dyn = obs->sizemax;
        }
    }
}


