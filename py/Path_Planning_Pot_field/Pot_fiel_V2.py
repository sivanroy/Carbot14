#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Created on Sun Mar  6 15:14:50 2022

@author: thomasclayson
"""
import numpy as np
import matplotlib.pyplot as plt

from sympy import *
from sympy.geometry import *

def calc_AttractivePotential(x,y,x_goal,y_goal,R):
    
    ALPHA = 2        # Scaling factor attractive potential
    d_limit = 2      # Composite potential field limit
    
    # Distance to goal
    d = np.hypot(x - x_goal , y - y_goal)
    
    if d <= d_limit:
        
        Fx = ALPHA*(x - x_goal)
        Fy = ALPHA*(y - y_goal)
        
        return  Fx,Fy
    
    else:
        
        Fx = (d_limit * ALPHA * (x - x_goal))/(d)
        Fy = (d_limit * ALPHA * (y - y_goal))/(d)
        
        return  Fx,Fy

def calc_RepulsivePotential(x,y,x_obst,y_obst,R):
    
    ETHA = 0.2       # Scaling factor repulsive potential
    front_obst = 5   # Influence dimension of obstacle
    
    radius_obst = 1       # Obstacle radius
    
    Fx = 0.0
    Fy = 0.0
    
    for i in range(x_obst): 
        
        # Distance to obstacle i
        d = np.hypot(x - x_obst[i] , y - y_obst[i])
        
        # If in robot range
        if d <= R:
            
            if radius_obst < d <= front_obst:
                
                Fx += ETHA * (1/front_obst - 1/d) * 1/(d*d) * (x-x_obst)/2
                Fy += ETHA * (1/front_obst - 1/d) * 1/(d*d) * (y-y_obst)/2
            
        # If not in robot range
        else:
            
            continue
        
    return Fx,Fy
        
        
def main(x,y,xg,yg,xo,yo,R):
    
    F_att_x,F_att_y = calc_AttractivePotential(x,y,xg,yg,d_limit,R)
    F_rep_x,F_rep_y = calc_RepulsivePotential(x,y,xo,yo,front_obst,R)
    
    F_tot_x = F_att_x + F_rep_x
    F_tot_y = F_att_y + F_rep_y
    
    gain_vx = 1
    gain_vy = 1
    
    v_x = gain_vx * F_tot_x
    v_y = gain_vy * F_tot_y
       
    return

main();

 