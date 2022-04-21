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
from math import *

dt = 0.001

d_limit = 0.20      # Composite potential field limit
ALPHA = 1/d_limit   # Scaling factor attractive potential


front_obst = 0.5 # Influence dimension of obstacle
ETHA = 0.025 #front_obst**A      # Scaling factor repulsive potential

tau = 5

gain_vx = tau
gain_vy = tau

MAXX = 1

#créer un potential field correspondant a la forme de notre robot ??

def calc_AttractivePotential(x,y,x_goal,y_goal,ALPHA):
    # Distance to goal
    #ALPHA = ALPHA/d_limit
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
    #ETHA = 0.02      # Scaling factor repulsive potential
    #front_obst = 0.1 # Influence dimension of obstacle
    #radius_obst = 0.01       # Obstacle radius
    Fx = 0.0
    Fy = 0.0
    for i in range(len(x_obst)): 
        # Distance to obstacle i
        d = sqrt((x - x_obst[i])**2 + (y - y_obst[i])**2)
        # If in robot range
        if d <= R:
            if d <= front_obst:
                if abs(x-x_obst[i]) <= 0.000001:
                    Fx += MAXX;
                else:
                    Fx += ETHA * (1/front_obst - 1/d) * 1/(d*d) * (x-x_obst[i])/2

                if abs(y-y_obst[i]) <= 0.000001:
                    Fy += MAXX;
                else :
                    Fy += ETHA * (1/front_obst - 1/d) * 1/(d*d) * (y-y_obst[i])/2
            else:
                Fx += 0
                Fy += 0
    if (Fx>MAXX):
        Fx = MAXX
    if(Fy> MAXX):
        Fy=MAXX
    return Fx,Fy
      
  
        
def main_pot_force(x,y,xg,yg,xo,yo,R):
    F_att_x,F_att_y = calc_AttractivePotential(x,y,xg,yg,R)
    F_rep_x,F_rep_y = calc_RepulsivePotential(x,y,xo,yo,R)
    
    F_tot_x = (F_att_x + F_rep_x) * -1
    F_tot_y = (F_att_y + F_rep_y) * -1
    #print("F_rep : {} ; {} \n".format(F_rep_x,F_rep_y))
    #print("F_att : {} ; {} \n".format(F_att_x,F_att_y))


    v_x = gain_vx * F_tot_x # [m/s]
    v_y = gain_vy * F_tot_y # [m/s]
    
    #print(F_tot_x,F_tot_y)
    
    #theta = arctan(F_tot_y/F_tot_x) * 180/np.pi # [degrees]
    return v_x, v_y



def travel_loop(dt,steps,xo,y0):    
    x,y   = 2.80,1.3
    R     = 0.50
    
    x_pos = np.zeros(steps)
    y_pos = np.zeros(steps)
    
    x_pos[0] = x
    y_pos[0] = y
    
    v_x   = np.zeros(steps-1)
    v_y   = np.zeros(steps-1)
    theta = np.zeros(steps-1)
          
    for i in range(steps-1):
        if (i <steps//2):                
            #xg,yg = 2.5,1.5
            xg,yg = 1.3,1.75
            if (i==0):
                print("goal1")
        else:
            if (i== steps//2):
                print("goal2")
            xg,yg = 1,1
        v_x[i], v_y[i] = main_pot_force(x, y, xg, yg, xo, yo, R)
        #print(sqrt(v_x[i]**2+v_y[i]**2))
        #if( (v_x[i]*dt + v_y[i]*dt <= 0.0001) and i> steps/2 ):
        #    continue
        new_x = x + v_x[i]*dt
        new_y = y + v_y[i]*dt
        x = new_x
        y = new_y
        x_pos[i+1] = new_x
        y_pos[i+1] = new_y      
    return x_pos,y_pos
    
def lines(xy1,xy2,dx):
    d = sqrt((xy1[0]-xy2[0])**2+(xy1[1]-xy2[1])**2)
    N = int(max(d//dx,3))
    if ( (xy2[0] - xy1[0]) == 0) :
        pointsx = []
        pointsy = []
        for i in range(N):
            y1 = xy1[1];y2 = xy2[1]
            dy = y2-y1
            step = dy/(N-1)
            pointsy.append(y1+step*i)
            pointsx.append(xy1[0])
        return pointsx,pointsy
        
    else :
        a = (xy2[1]-xy1[1]) / (xy2[0] - xy1[0]);
        b = xy1[1] - a*xy1[0]
        x1 = xy1[0]; x2 = xy2[0]
        pointsx = []
        pointsy = []
        for i in range(N):
            dx = x2-x1
            step = dx / (N-1)
            pointsx.append(x1+step*i)
            y = a*(x1+step*i)+b
            pointsy.append(y)
        return pointsx,pointsy       
    


        
    
def obstacles():
    xo = []
    yo = []
    #limitsx = [[0,3.00],[3.00,3.00],[3.00,0],[0,0],[0,0.30],[1,1.2],[1.8,2]]
    #limitsy = [[0,0],[0,2.00],[2.00,2.00],[2.00,0],[0.30,0],[0.4,.70],[.70,1]]
    limitsx = [[0.56,2.43],[3.00,3.0], [3.00,2.555],[.44,0],[1.175,1.825]  ,[0,0],\
               [.51,0],[2.49,3.0],\
               [1.45,1.5],[1.55,1.5],\
               [2.898,2.898],[.102,.102],\
               [.45,1.17],[1.830,2.55],\
               #[1.48,1.48],[1.52,1.52],\
               #[.45,.45],[1.17,1.17],\
               #[1.83,1.83],[2.55,2.55]\
                   ];
    limitsy = [[0,0],  [0.57,2.00],   [2.00,2.00],[2.00,2.00],[2.00,2.00],  [2.00,0.57],\
               [0,.51],[0,.51],\
               [1.95,1.7],[1.95,1.65],\
               [.825,.675],[.825,.675],\
               [1.914,1.914],[1.914,1.914],\
               #[1.95,1.7],[1.95,1.7],\
               #[1.914,2],[1.914,2],\
               #[1.914,2],[1.914,2]\
                   ]; 
    density = .05
    for i in range (len(limitsx)):
    #for i in range (1):
        xs = limitsx[i]
        ys = limitsy[i]
        xp,yp = lines([xs[0],ys[0]],[xs[1],ys[1]],density)
        xo.extend(xp)
        yo.extend(yp)
    print("size of obstacles : {}".format(len(xo)))
    return xo,yo

xo,yo = obstacles()
x_pos,y_pos = travel_loop(dt, 1, xo, yo)

x_obst = np.linspace(.01,1,100)
F_rep = []
for xi in x_obst :
    Fx,Fy = calc_RepulsivePotential(0, 0, [xi], [0], 1)
    a = sqrt(Fx**2+Fy**2)
    F_rep.append(Fx)
    
plt.plot(x_obst,F_rep)
plt.title("Repulsive force to a point obstacle",fontsize = 20)
plt.ylabel("Repulsive Force [-]",fontsize = 18)
plt.xlabel("distance [m]",fontsize = 18)
plt.yticks(fontsize=15)
plt.xticks(fontsize=15)
plt.show()

F_goal = []
x_goal = np.linspace(0,.25,100)
for xi in x_goal :
    Fx,Fy = calc_AttractivePotential(0, 0, xi, 0,ALPHA)
    a = sqrt(Fx**2+Fy**2)
    if (a> .98) : a = .98
    F_goal.append(a)
    
plt.plot(x_goal,F_goal)
plt.yticks(fontsize=15)
plt.xticks(fontsize=15)
plt.ylabel("Attractive Force [-]",fontsize = 18)
plt.xlabel("distance [m]",fontsize = 18)
plt.title("Attractive force to a goal point",fontsize = 20)
plt.show()

fig = plt.figure(figsize =(10, 7))
plt.plot(x_pos,y_pos,'bo')
#plt.plot(x_pos[-1],y_pos[-1],'bo',c='red')
plt.plot(xo,yo,'bo',c='r')
#plt.plot(2.5,1.5,'bo',c='green')
plt.plot(1.3,1.75,'bo',c='green')
plt.plot(1,1, 'bo',c = 'green')
plt.grid()

plt.ylabel("Y pos [m]",fontsize = 18)
plt.xlabel("X pos [m]",fontsize = 18)
plt.title("Path to 2 goals from camp",fontsize = 20)
plt.show()


fig = plt.figure(figsize =(14, 9))
#ax = plt.axes(projection='3d')
ax = plt.gca()
xl = 100; yl=100
de = 0
X = np.zeros([xl,yl])
Y = np.zeros([xl,yl])
x = (np.linspace(0+de,3-de,xl))
y=  (np.linspace(0+de,2-de,yl))
pmap = np.zeros([xl,yl])
for i in range (len(x)):
    print("{} / {}".format(i,xl))
    for j in range (len(y)):
        F_rep_x,F_rep_y = calc_RepulsivePotential(x[i], y[j], xo, yo, front_obst)
        X[i][j] = x[i]
        Y[i][j] = y[j]
        pmapi = sqrt(F_rep_x**2+F_rep_y**2)
        #print("pmapi : {}".format(pmapi))
        pmap[i][j] = min(pmapi,MAXX)
print("end")

ax.pcolormesh(X, Y, pmap , cmap='viridis', edgecolor='none')
x_ticks = np.arange( 0, 3, 0.1 ) # minor tick for every 2 units
y_ticks = np.arange(0,2,0.1)
ax.set_xticks( x_ticks )
ax.set_yticks( y_ticks)
plt.grid()
#ax.set_title('Potential Field');
plt.show()

ax = plt.axes(projection='3d')
ax.plot_surface(X, Y, pmap ,rstride=1, cstride=1,cmap='viridis', edgecolor='none')
ax.view_init(70, -30)
plt.grid()
plt.show()



"""
    limitsx = {{0.55,2.45},{3.00,3.0}, {3.00,2.555},{.445,0},{1.175,1.825}  ,{0,0},\
               {.51,0},{2.49,3.0},\
               {1.5,1.5},
               {2.868,2.868},{.102,.102},\
               {.45,1.17},{1.830,2.55},\
                   };
    limitsy = {{0,0},  {0.55,2.00},   {2.00,2.00},{2.00,2.00},{2.00,2.00},  {2.00,0.55},\
               {0,.51},{0,.51},\
               {1.95,1.7},
               {.825,.675},{.825,.675},\
               {1.914,1.914},{1.914,1.914},\
                   }; 
"""