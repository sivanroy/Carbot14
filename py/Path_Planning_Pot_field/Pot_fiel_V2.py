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
    
    ALPHA   = 1     # Scaling factor attractive potential
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
    
    radius_obst = 0.001       # Obstacle radius
    
    Fx = 0.0
    Fy = 0.0
    
    for i in range(len(x_obst)): 
        
        # Distance to obstacle i
        d = np.hypot(x - x_obst[i] , y - y_obst[i])
        
        # If in robot range
        if d <= R:
            
            if radius_obst < d <= front_obst:
                
                Fx += ETHA * (1/front_obst - 1/d) * 1/(d*d) * (x-x_obst[i])/2
                Fy += ETHA * (1/front_obst - 1/d) * 1/(d*d) * (y-y_obst[i])/2
            
        # If not in robot range
        else:
            
            continue
        
    return Fx,Fy
        
        
def main_pot_force(x,y,xg,yg,xo,yo,R):
    
    F_att_x,F_att_y = calc_AttractivePotential(x,y,xg,yg,R)
    F_rep_x,F_rep_y = calc_RepulsivePotential(x,y,xo,yo,R)
    
    F_tot_x = (F_att_x + F_rep_x) * -1
    F_tot_y = (F_att_y + F_rep_y) * -1
    
    gain_vx = 0.01
    gain_vy = 0.01
    
    v_x = gain_vx * F_tot_x # [m/s]
    v_y = gain_vy * F_tot_y # [m/s]
    
    theta = np.arctan(F_tot_y/F_tot_x) * 180/np.pi # [degrees]

    return theta, v_x, v_y

def travel_loop(t0,tf,steps):
    
    dt = int((tf-t0)/steps)
    
    x,y   = 0,0
    xg,yg = 1,1
    xo    = [0.5]
    yo    = [0.5]
    R     = 10
    
    x_pos = np.zeros(steps)
    y_pos = np.zeros(steps)
    
    x_pos[0] = x
    y_pos[0] = y
    
    v_x   = np.zeros(steps-1)
    v_y   = np.zeros(steps-1)
    theta = np.zeros(steps-1)
      
    for i in range(t0,tf-1,dt):
              
        theta[i], v_x[i], v_y[i] = main_pot_force(x, y, xg, yg, xo, yo, R)
        
        new_x = x + v_x[i]*dt
        new_y = y + v_y[i]*dt
        
        x = new_x
        y = new_y
        
        x_pos[i+1] = new_x
        y_pos[i+1] = new_y
        
    return x_pos,y_pos,theta
        
x_pos,y_pos,theta = travel_loop(0, 100, 100)

print(x_pos)

plt.plot(x_pos,y_pos,'bo')



 