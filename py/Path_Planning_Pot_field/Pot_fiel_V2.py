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

ETHA = 0.2       # Scaling factor repulsive potential
ALPHA = 2        # Scaling factor attractive potential
d_limit = 2      # Composite potential field limit
front_obst = 5   # Influence dimension of obstacle
max_pot_value = 5 # Maximum value of the potential representing objects

R = 0.02         # obstacle radius TO BE MODIFY

grid_x_size = 3 # size [m]
grid_y_size = 2 # size [m]

res = 0.01      # meshing resolution [mm]

def calc_AttractivePotential(x,y,x_goal,y_goal,d_limit):
    
    # Distance to goal
    d = np.hypot(x - x_goal , y - y_goal)
    
    if d <= d_limit:
        
        return  0.5*ALPHA*d*d
    
    else:
        return (d_limit*ALPHA*d)-(0.5*ALPHA*d_limit*d_limit) 

def calc_RepulsivePotential(x,y,x_obst,y_obst,front_obst):
    
    # Search the nearest obst
    minid = -1
    dmin = float("inf")
    for i, _ in enumerate(x_obst):
        d = np.hypot(x - x_obst[i], y - y_obst[i])
        if dmin >= d:
            dmin = d
            minid = i
    
    # Distance to obstacle
    d = np.hypot(x - x_obst[minid] , y - y_obst[minid])
    
    if R < d <= front_obst:
        
        pot =  0.5*ETHA*(1/d-1/front_obst)**2
        
        if pot < max_pot_value:
            
            return pot
        else:
            
            return max_pot_value
    
    elif d < R:
        
        return max_pot_value
        
    else:
        return 0
        
def main():
    
    x = np.arange(0,grid_x_size,res)
    y = np.arange(0,grid_y_size,res)

    xg,yg = 2.5,0.25
    xo,yo = [0,0.5],[0,0.5]

    X,Y = np.meshgrid(x,y)
    pmap = np.zeros((len(y),len(x)))
    
    for i in range(len(x)):
        for j in range(len(y)):
            
            U_att = calc_AttractivePotential(x[i],y[j],xg,yg,d_limit)
            U_rep = calc_RepulsivePotential(x[i],y[j],xo,yo,front_obst)

            U_tot = U_rep

            pmap[j][i] = U_tot
        
              
    fig = plt.figure(figsize =(14, 9))

    ax = plt.axes(projection='3d')
    ax = plt.axes(projection='3d')
    ax = plt.axes(projection='3d')
    ax.plot_surface(X, Y, pmap ,rstride=1, cstride=1,cmap='viridis', edgecolor='none')
    ax.set_title('Potential Field');
    return

main();
    
    