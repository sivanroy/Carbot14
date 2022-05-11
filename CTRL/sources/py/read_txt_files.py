import matplotlib.pyplot as plt
from math import pi
import numpy as np


def read_txt_file(path_file, nData):

    f = open(path_file, "r")
    lines = f.readlines()

    Data = []
    for i in range(nData):
        Data.append([])

    for line in lines:
        split_line = line.split(",")
        if len(split_line) != nData:
            break
        for i in range(nData):
            Data[i].append(float(split_line[i]))

    return Data


def plot_mlc_data():

    Data = read_txt_file("../../build/mlc_data.txt", 7)

    plt.plot(Data[0], Data[3], label="d_ref")
    plt.plot(Data[0], Data[4], label="d_mes")
    plt.xlabel("t [s]")
    plt.ylabel("d [m]")
    plt.legend()
    plt.grid()
    plt.show()

    plt.plot(Data[0], Data[5], label="th_ref")
    plt.plot(Data[0], Data[6], label="th_mes")
    plt.xlabel("t [s]")
    plt.ylabel("th [rad]")
    plt.legend()
    plt.grid()
    plt.show()

    plt.plot(Data[0], Data[1], label="r_sp_ref")
    plt.plot(Data[0], Data[2], label="l_sp_ref")
    plt.xlabel("t [s]")
    plt.ylabel("w [rad]")
    plt.legend()
    plt.grid()
    plt.show()


    
def open_loop():
    
    Data = read_txt_file("../../build/llc_data.txt", 5)
    
    moy_list = []
    moy = 0
    for i in range (len(Data[0])):
        if Data[0][i] > 0.4:
            moy_list.append(Data[3][i])
            moy = moy + Data[3][i]
            
    moy = moy/len(moy_list)
    print(moy)
    
    moy_95 = moy*0.95
    
    t_95 = 0
    for i in range (len(Data[0])):
        if Data[4][i] >= moy_95:
            t_95 = Data[0][i]
            break
    print(t_95)
    
    plt.plot(Data[0], Data[1], label=r"$cmd_{ref}$", linewidth=2)
    plt.plot(Data[0], Data[3], label=r"$\omega_{mes,r}$", linewidth=1.2)
    plt.xlabel("Time [s]")
    plt.ylabel(r"$\omega$ [rad/s]")
    plt.title("R")
    plt.xlim(0, 0.05)
    #plt.ylim(-1, 6)
    #plt.axis([0, 10, -6, 6])
    plt.legend()
    plt.grid()
    #plt.savefig("speed_profile_zoom.pdf", format="pdf")

    plt.show()
    
    plt.plot(Data[0], Data[2], label=r"$cmd_{ref}$", linewidth=2)
    plt.plot(Data[0], Data[4], label=r"$\omega_{mes,r}$", linewidth=1.2)
    plt.xlabel("Time [s]")
    plt.ylabel(r"$\omega$ [rad/s]")
    plt.title("L")
    plt.xlim(0, 0.02)
    #plt.ylim(-1, 6)
    #plt.axis([0, 10, -6, 6])
    plt.legend()
    plt.grid()
    #plt.savefig("speed_profile_zoom.pdf", format="pdf")

    plt.show()



def plot_llc_data():

    Data = read_txt_file("../../build/llc_data.txt", 7)
    Data2 = read_txt_file("../../build/llc_data2.txt", 9)
    
    moy_list = []
    moy = 0
    for i in range (len(Data[0])):
        if Data[0][i] > 0.2:
            moy_list.append(Data[4][i])
            moy = moy + Data[4][i]
            
    moy = moy/len(moy_list)
    print(moy)
    
    moy_95 = moy*0.95
    
    t_95 = 0
    for i in range (len(Data[0])):
        if Data[4][i] >= moy_95:
            t_95 = Data[0][i]
            break
    print(t_95)

    
    fig,(ax1,ax2) = plt.subplots(2,sharex=True)
    #plt.suptitle("Wheels speed profile")
    

    ax1.plot(Data[0], Data[1], label=r"$\omega_{ref}$", linewidth=2)
    ax1.plot(Data[0], Data[3], label=r"$\omega_{mes,r}$", linewidth=1.2)
    #ax1.set(xlabel="Time [s]")
    ax1.set(ylabel=r"$\omega_r$ [rad/s]")
    ax1.set(title="Right wheel speed profile")
    #plt.xlim(1.95, 2.1)
    #plt.ylim(-1, 6)
    #plt.axis([0, 10, -6, 6])
    ax1.legend()
    ax1.grid()
    #plt.savefig("speed_profile_zoom.pdf", format="pdf")
    #plt.show()
    
    ax2.plot(Data[0], Data[2], label=r"$\omega_{ref}$", linewidth=2)
    ax2.plot(Data[0], Data[4], label=r"$\omega_{mes,l}$", linewidth=1.2,c='r')
    #plt.plot(Data[0], Data[3], label=r"$\omega_{mes,r}$", linewidth=1.2,ls=":",c="g")
    ax2.set(xlabel="Time [s]")
    ax2.set(ylabel="$\omega_l$ [rad/s]")
    ax2.set(title="Left wheel speed profile")
    #plt.xlim(0, 0.05)
    #plt.ylim(-1, 6)
    #plt.axis([0, 10, -6, 6])
    ax2.legend()
    ax2.grid()
    plt.savefig("plot_llc.pdf", format="pdf")
    plt.show()
    return
    plt.plot(Data[0], Data[1], label=r"$\omega_{ref}$", linewidth=2)
    plt.plot(Data[0], Data[5], label=r"$\omega_{mes,r}$", linewidth=1.2)
    plt.xlabel("Time [s]")
    plt.ylabel(r"$\omega$ [rad/s]")
    plt.title("R wheel speed profile")
    #plt.xlim(0, 0.05)
    #plt.ylim(-1, 6)
    #plt.axis([0, 10, -6, 6])
    plt.legend()
    plt.grid()
    #plt.savefig("speed_profile_zoom.pdf", format="pdf")

    plt.show()
    
    plt.plot(Data[0], Data[2], label=r"$\omega_{ref}$", linewidth=2)
    plt.plot(Data[0], Data[6], label=r"$\omega_{mes,l}$", linewidth=1.2)
    plt.xlabel("Time [s]")
    plt.ylabel(r"$\omega$ [rad/s]")
    plt.title("L wheel speed profile")
    #plt.xlim(0, 0.05)
    #plt.ylim(-1, 6)
    #plt.axis([0, 10, -6, 6])
    plt.legend()
    plt.grid()
    #plt.savefig("speed_profile_zoom.pdf", format="pdf")
    plt.show()
    
    plt.plot(Data2[0], Data2[1], label=r"r_Pout", linewidth=2)
    plt.plot(Data2[0], Data2[3], label=r"r_Iout$", linewidth=1.2)
    plt.plot(Data2[0], Data2[5], label=r"r_cmd$", linewidth=1.2)
    plt.xlabel("Time [s]")
    plt.ylabel(r"[-]")
    plt.title("llc R")
    #plt.xlim(0, 0.05)
    #plt.ylim(-1, 6)
    #plt.axis([0, 10, -6, 6])
    plt.legend()
    plt.grid()
    #plt.savefig("speed_profile_zoom.pdf", format="pdf")
    plt.show()
    
    plt.plot(Data2[0], Data2[2], label=r"l_Pout", linewidth=2)
    plt.plot(Data2[0], Data2[4], label=r"l_Iout$", linewidth=1.2)
    plt.plot(Data2[0], Data2[6], label=r"l_cmd$", linewidth=1.2)
    plt.xlabel("Time [s]")
    plt.ylabel(r"[-]")
    plt.title("llc L")
    #plt.xlim(0, 0.05)
    #plt.ylim(-1, 6)
    #plt.axis([0, 10, -6, 6])
    plt.legend()
    plt.grid()
    #plt.savefig("speed_profile_zoom.pdf", format="pdf")
    plt.show()
    
    plt.plot(Data[0], Data[2], label=r"$\omega_{ref}$", linewidth=2)
    plt.plot(Data[0], Data[6], label=r"$\omega_{mes,l}$", linewidth=1.2)
    plt.plot(Data[0], Data[5], label=r"$\omega_{mes,r}$", linewidth=1.2)
    plt.xlabel("Time [s]")
    plt.ylabel(r"$\omega$ [rad/s]")
    plt.title("Wheels speed profile")
    #plt.xlim(0, 0.05)
    #plt.ylim(-1, 6)
    #plt.axis([0, 10, -6, 6])
    plt.legend()
    plt.grid()
    #plt.savefig("speed_profile_zoom.pdf", format="pdf")
    plt.show()
    
    plt.plot(Data[0], Data[0], label="P")
    plt.plot(Data[0], Data[1], label="I")
    plt.plot(Data[0], Data[2], label="kphi")
    plt.xlabel("t [s]")
    plt.ylabel("cmd")
    plt.title("cmd")
    plt.legend()
    plt.grid()
    plt.show()
    
    """
    m = 0
    T = 0
    i = 0
    for t in Data[3]:
        
        if t == -14.4:
            m = Data[7][i]
        i += 1
    i = 0
    for t in Data[3]:
        if Data[7][i] >= m*0.95:
            T = t + 14.5
            break
        i += 1
        
    print(m)
    print(T)
    """ 
    

def plot_lpf_data():

    Data = read_txt_file("../../build/lpf_data.txt", 8)

    plt.plot(Data[0], label=r"mes", linewidth=2)
    plt.plot(Data[4], label=r"lpf", linewidth=1.2)
    plt.xlabel("Time [s]")
    plt.ylabel(r"$\omega$ [rad/s]")
    plt.title("Enc r")
    #plt.xlim(0, 0.05)
    plt.ylim(-20, 20)
    #plt.axis([0, 10, -6, 6])
    plt.legend()
    plt.grid()
    #plt.savefig("speed_profile_zoom.pdf", format="pdf")
    plt.show()
    
    plt.plot(Data[1], label=r"mes", linewidth=2)
    plt.plot(Data[5], label=r"lpf", linewidth=1.2)
    plt.xlabel("Time [s]")
    plt.ylabel(r"$\omega$ [rad/s]")
    plt.title("Enc l")
    #plt.xlim(150, 200)
    plt.ylim(-20, 20)
    #plt.axis([0, 10, -6, 6])
    plt.legend()
    plt.grid()
    #plt.savefig("speed_profile_zoom.pdf", format="pdf")
    plt.show()
    
    plt.plot(Data[2], label=r"mes", linewidth=2)
    #plt.plot(Data[6], label=r"lpf", linewidth=1.2)
    plt.xlabel("Time [s]")
    plt.ylabel(r"$\omega$ [rad/s]")
    plt.title("Odo r")
    #plt.xlim(0, 0.05)
    plt.ylim(-20, 20)
    #plt.axis([0, 10, -6, 6])
    plt.legend()
    plt.grid()
    #plt.savefig("speed_profile_zoom.pdf", format="pdf")
    plt.show()
    
    plt.plot(Data[3], label=r"mes", linewidth=2)
    #plt.plot(Data[7], label=r"lpf", linewidth=1.2)
    plt.xlabel("Time [s]")
    plt.ylabel(r"$\omega$ [rad/s]")
    plt.title("Odo l")
    #plt.xlim(0, 0.05)
    plt.ylim(-20, 20)
    #plt.axis([0, 10, -6, 6])
    plt.legend()
    plt.grid()
    #plt.savefig("speed_profile_zoom.pdf", format="pdf")
    plt.show()

def plot_enc_data():
    
    Data = read_txt_file("../../build/llc_data.txt", 5)

    plt.plot(Data[0], Data[1], label=r"$r$", linewidth=2)
    plt.plot(Data[0], Data[2], label=r"$l$", linewidth=1.2)
    plt.xlabel("Time [s]")
    plt.ylabel(r"$Ticks$")
    plt.title("Enc")
    #plt.xlim(9.8, 10)
    #plt.ylim(-1, 6)
    #plt.axis([0, 10, -6, 6])
    plt.legend()
    plt.grid()
    #plt.savefig("speed_profile_zoom.pdf", format="pdf")
    plt.show()
    
    plt.plot(Data[0], Data[3], label=r"$r$", linewidth=2)
    plt.plot(Data[0], Data[4], label=r"$l$", linewidth=1.2)
    plt.xlabel("Time [s]")
    plt.ylabel(r"$Ticks$")
    plt.title("Odo")
    #plt.xlim(9.8, 10)
    plt.ylim(-1000, 1000)
    #plt.ylim(-1, 6)
    #plt.axis([0, 10, -6, 6])
    plt.legend()
    plt.grid()
    #plt.savefig("speed_profile_zoom.pdf", format="pdf")
    plt.show()


def plot_mlc_opti():

    Data = read_txt_file("../../build/mlc_data.txt", 5)
    
    fig,(ax1,ax2) = plt.subplots(2,sharex=True)
    
    ax1.plot(Data[0],Data[1],label=r"$v_{ref}$")#vref
    ax1.plot(Data[0],Data[3],label=r"$v_{mes}$")#v
    ax1.set(title=r"Speed profile")
    ax1.set(ylabel="v [m/s]")
    ax1.set(xlim=[0,10])
    ax1.legend()
    ax1.grid()
    
    ax2.plot(Data[0],Data[2],label=r"$\theta_{ref}$")#thetaref
    ax2.plot(Data[0],Data[4],label=r"$\theta_{mes}$")#theta
    ax2.set(title=r"$\theta$ profile")
    ax2.set(ylabel=r"$\theta$ [rad]")
    ax2.set(xlabel="time [s]")
    ax2.legend()
    ax2.grid()
    #plt.show()

    plt.savefig("mlcPF output.pdf", format="pdf")                          
    
    return

    plt.plot(Data[0], Data[1], label="x_g")
    plt.plot(Data[0], Data[3], label="x")
    plt.xlabel("t [s]")
    plt.ylabel("x [m]")
    plt.legend()
    plt.grid()
    plt.show()

    plt.plot(Data[0], Data[9], label="ref")
    plt.plot(Data[0], Data[11], label="mes")
    plt.xlabel("t [s]")
    plt.ylabel("w [rad/s]")
    plt.title("R speed")
    plt.legend()
    plt.grid()
    plt.show()


def plot_mlcPF_data():

    Data = read_txt_file("../../build/mlcPF_data.txt", 9)
    """
    plt.plot(Data[0], Data[1], label="th_ref")
    plt.plot(Data[0], Data[2], label="th_mes")
    plt.xlabel("t [s]")
    plt.ylabel("th [rad]")
    plt.title("th response")
    plt.legend()
    plt.grid()
    plt.show()
    
    plt.plot(Data[0], Data[7], label="v_ref")
    plt.plot(Data[0], Data[8], label="v_mes")
    plt.xlabel("t [s]")
    plt.ylabel("v [m/s]")
    plt.title("v response")
    plt.legend()
    plt.grid()
    plt.show()
    """
    fig, (ax1, ax2) = plt.subplots(2, sharex=True)
    fig.suptitle("Orientation and speed profiles")
    ax1.plot(Data[0], Data[1], label=r"$\theta_{ref}$")
    ax1.plot(Data[0], Data[2], label=r"$\theta$")
    ax1.set(ylabel='Angle [rad]')
    ax1.legend()
    ax1.grid()
    ax2.plot(Data[0], Data[7], label=r"$v_{ref}$", color='b', linewidth=2)
    ax2.plot(Data[0], Data[8], label=r"$v_{mes}$", color='r', linewidth=1.2)
    ax2.set(xlabel='Time [s]', ylabel='Speed [m/s]')
    ax2.legend()
    ax2.grid()
    #plt.savefig("mlcPF_profiles.pdf", format="pdf")
    plt.show()


    plt.plot(Data[3], Data[4], label="Trajectory")
    plt.plot(0, 0, 'go', label="Initial position")
    plt.plot(0.6, 0, 'ro', label=r"Expected position of the change of orientation")
    plt.xlabel("x [m]")
    plt.ylabel("y [m]")
    plt.title("Measured position of the robot")
    #plt.ylim(-0.1, 0.6)
    plt.axis("equal")
    plt.legend()
    plt.grid()
    plt.savefig("mlcPF_pos.pdf", format="pdf")
    plt.show()


def plot_rpl_data():

    Data = read_txt_file("../../build/rpl_data.txt", 2)

    Map = read_txt_file("../../build/icp1_data.txt", 2)
    
    plt.plot(Map[0], Map[1], label="Map")
    plt.plot(Data[0], Data[1], 'ro', label="rpl_data")
    plt.xlabel("x [m]")
    plt.ylabel("y [m]")
    plt.title("rpl")
    plt.ylim(-0.1, 2.1)
    plt.xlim(-0.1, 3.1)
    #plt.axis("equal")
    plt.legend()
    plt.grid()
    #plt.savefig("rpl_data2.pdf", format="pdf")
    plt.show()
    
def plot_op_data():

    Data = read_txt_file("../../build/op_data.txt", 2)

    Map = read_txt_file("../../build/icp1_data.txt", 2)
    
    plt.plot(Map[0], Map[1], label="Map")
    plt.plot(Data[0], Data[1], 'ro', label="op_data")
    plt.xlabel("x [m]")
    plt.ylabel("y [m]")
    plt.title("op")
    plt.ylim(-0.1, 2.1)
    plt.xlim(-0.1, 3.1)
    #plt.axis("equal")
    plt.legend()
    plt.grid()
    #plt.savefig("op_data2.pdf", format="pdf")
    plt.show()

def lines(xy1,xy2,dx):
    d = np.sqrt((xy1[0]-xy2[0])**2+(xy1[1]-xy2[1])**2)
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


def plot_mp_data():
    

    Data = read_txt_file("../../build/mp_data.txt", 6)

    Map = read_txt_file("../../build/icp1_data.txt", 2)
    xo,yo=obstacles()
    
    plt.plot([1.55,1.85,1.85,1.55,1.55],[.6,.6,.8,.8,.6],label="Opponent",linewidth=4,c='r')
    #plt.plot(Map[0], Map[1], label="Map",c="b")
    plt.plot(xo,yo,"o",c='r',markersize = 4,label="Walls")
    plt.plot(Data[0], Data[1], 'ro', label="Path", markersize = 3,c='b')
    plt.plot([1,.75,2.5],[1.55,.5,1.4],'o',c='g',markersize = 6,label="Goals")
    plt.xlabel("x [m]")
    plt.ylabel("y [m]")
    plt.title("Potential field path planning")
    plt.xlim(-0.1, 3.1)
    plt.ylim(-0.1, 2.1)
    #plt.axis("equal")
    plt.legend()
    plt.grid()
    plt.savefig("mp_data2.pdf", format="pdf")
    plt.show()
    
    plt.plot(Data[5], Data[3], 'r', label="w")
    plt.xlabel("t [s]")
    plt.ylabel("w [rad/s]")
    plt.title("w")
    #plt.axis("equal")
    plt.legend()
    plt.grid()
    #plt.savefig("mp_data2.pdf", format="pdf")
    plt.show()
    
    plt.plot(Data[5], Data[4], 'r', label="v")
    plt.xlabel("t [s]")
    plt.ylabel("v [m/s]")
    plt.title("v")
    #plt.axis("equal")
    plt.legend()
    plt.grid()
    #plt.savefig("mp_data2.pdf", format="pdf")
    plt.show()
    
    print("x = ", Data[0][len(Data[0])-1])
    print("y = ", Data[1][len(Data[0])-1])
    print("th = ", Data[2][len(Data[0])-1])
    
def plot_icp_data():
    
    Data1 = read_txt_file("../../build/icp1_data.txt", 2)
    Data2 = read_txt_file("../../build/icp2_data.txt", 2)
    Data3 = read_txt_file("../../build/icp3_data.txt", 2)
    
    plt.plot(Data1[0], Data1[1], 'ro', label="Map point cloud ", markersize = 2)
    plt.plot(Data2[0], Data2[1], 'bo', label="rplidar point cloud", markersize = 2)
    plt.plot(Data3[0], Data3[1], 'o', marker='x', label="Recalibrated point cloud")
    plt.plot(Data2[0][0], Data2[1][0], 'go', label="first rplidar point", markersize = 4)
    plt.plot(Data2[0][len(Data2[0])-1], Data2[1][len(Data2[0])-1], 'o', label="last rplidar point", markersize = 4)

    plt.xlabel("x [m]")
    plt.ylabel("y [m]")
    plt.title("ICP algorithm")
    #plt.ylim(1.75, 2.05)
    #plt.xlim(1.75, 2.05)
    #plt.ylim(-0.02, 0.25)
    #plt.xlim(-0.02, 0.25)
    #plt.ylim(-1, 3)
    # plt.xlim(-1, 4)
    #plt.ylim(-0.1, 0.2)
    #plt.xlim(1.6, 2.8)
    #plt.axis("equal")
    plt.legend()
    plt.grid()
    #plt.savefig("icp_test_stat_5.pdf", format="pdf")
    plt.show()
    
    
def plot_rec_data():

    rec = read_txt_file("../../build/rec_data.txt", 6)
    mp = read_txt_file("../../build/mp_data.txt", 6)

    Map = read_txt_file("../../build/icp1_data.txt", 2)
    """
    ####################################################
    plt.plot(mp[5], mp[3], 'r', label="w")
    plt.xlabel("t [s]")
    plt.ylabel("w [rad/s]")
    plt.title("w")
    #plt.axis("equal")
    plt.legend()
    plt.grid()
    #plt.savefig("mp_data2.pdf", format="pdf")
    plt.show()
    
    plt.plot(mp[5], mp[4], 'r', label="v")
    plt.xlabel("t [s]")
    plt.ylabel("v [m/s]")
    plt.title("v")
    #plt.axis("equal")
    plt.legend()
    plt.grid()
    #plt.savefig("mp_data2.pdf", format="pdf")
    plt.show()
    
    plt.plot(mp[2], 'r', label="th mp")
    plt.ylabel("th [rad]")
    plt.title("th mp")
    #plt.axis("equal")
    plt.legend()
    plt.grid()
    #plt.savefig("mp_data2.pdf", format="pdf")
    plt.show()
    
    plt.plot(rec[2], 'r', label="th rec")
    plt.ylabel("th [rad]")
    plt.title("th rec")
    #plt.axis("equal")
    plt.legend()
    plt.grid()
    #plt.savefig("mp_data2.pdf", format="pdf")
    plt.show()
    """
    ####################################################
    plt.plot(Map[0], Map[1], label="Map")
    plt.plot(rec[3], rec[4], 'ro', label="mp_data", markersize = 4)
    plt.plot(rec[0], rec[1], 'bo', label="rec_data", markersize = 2)
    plt.xlabel("x [m]")
    plt.ylabel("y [m]")
    plt.title("mp vs rec")
    #plt.ylim(-0.1, 2.1)
    #plt.xlim(2, 3.1)
    #plt.axis("equal")
    plt.legend()
    plt.grid()
    #plt.savefig("mp_data2.pdf", format="pdf")
    plt.show()
    """
    ####################################################
    plt.plot(rec[0], 'r', label="x rec", markersize = 4)
    plt.plot(rec[3], 'b', label="x mp", markersize = 2)
    #plt.ylabel("th [rad]")
    #plt.title("th rec")
    #plt.axis("equal")
    plt.legend()
    plt.grid()
    #plt.savefig("mp_data2.pdf", format="pdf")
    plt.show()
    
    plt.plot(rec[1], 'r', label="y rec", markersize = 4)
    plt.plot(rec[4], 'b', label="y mp", markersize = 2)
    #plt.ylabel("th [rad]")
    #plt.title("th rec")
    #plt.axis("equal")
    plt.legend()
    plt.grid()
    #plt.savefig("mp_data2.pdf", format="pdf")
    plt.show()
    """
    err = 0
    c = 0
    for i in range(len(rec[0])):
        dx = rec[0][i] - rec[3][i]
        err += dx
        c += 1
        
    err = err/c
    print(err)
    print("th_rec = ", rec[2][0]*180/pi)
    print("th_mp = ", rec[5][0]*180/pi)
    
# err moy with v_ref = 0.1 : 0.021595764705882307

def plot_tau_data():

    Data = read_txt_file("../../build/tau_data.txt", 3)
    plt.plot(Data[0], Data[1], label="speed")
    plt.plot(Data[0], Data[2], label = "tau")
    plt.xlabel("t [s]")
    plt.ylabel("tau [-]")
    plt.title("tau")
    #plt.axis("equal")
    plt.legend()
    plt.grid()
    #plt.savefig("mp_data2.pdf", format="pdf")
    plt.show()
    
def plot_lidar_caract():
    
    Data = read_txt_file("../../build/lidar_caract_data.txt", 3)
    Map = read_txt_file("../../build/icp1_data.txt", 2)
    
    plt.plot(Map[0], Map[1], label="Map")
    plt.plot(Data[0], Data[1], label="pos")
    plt.xlabel("x")
    plt.ylabel("y")
    plt.title("recalib pos")
    #plt.axis("equal")
    plt.legend()
    plt.grid()
    plt.xlim(-0.1, 3.1)
    plt.ylim(-0.1, 2.1)
    #plt.savefig("mp_data2.pdf", format="pdf")
    plt.show()
    
    plt.plot(Data[0], label="x")
    plt.xlabel("x")
    plt.title("recalib x")
    #plt.axis("equal")
    plt.legend()
    plt.grid()
    #plt.savefig("mp_data2.pdf", format="pdf")
    plt.show()
    
    plt.plot(Data[1], label="y")
    plt.xlabel("y")
    plt.title("recalib y")
    #plt.axis("equal")
    plt.legend()
    plt.grid()
    #plt.savefig("mp_data2.pdf", format="pdf")
    plt.show()
    
    plt.plot(Data[2], label="th")
    plt.xlabel("th")
    plt.title("recalib th")
    #plt.axis("equal")
    plt.legend()
    plt.grid()
    #plt.savefig("mp_data2.pdf", format="pdf")
    plt.show()
    
    xlist = Data[0]
    ylist = Data[1]
    thlist = Data[2]
    
    xref = 3-0.133
    yref = 1.467
    thref = -pi
    
    n = len(xlist)-1
    moy_x = 0
    moy_y = 0
    moy_th = 0
    sd_x = 0
    sd_y = 0
    sd_th = 0
    
    for i in range(1,n+1):
        moy_x += xlist[i]
        moy_y += ylist[i]
        moy_th += thlist[i]
        
    moy_x = moy_x/n
    moy_y = moy_y/n
    moy_th = moy_th/n
    
    print(moy_x)
    print(moy_y)
    print(moy_th)
    
    for i in range(1,n+1):
        sd_x += (xlist[i] - moy_x)*(xlist[i] - moy_x)
        sd_y += (ylist[i] - moy_y)*(ylist[i] - moy_y)
        sd_th += (thlist[i] - moy_th)*(thlist[i] - moy_th)
        
    sd_x = np.sqrt(sd_x/n)
    sd_y = np.sqrt(sd_y/n)
    sd_th = np.sqrt(sd_th/n)
    
    print(sd_x)
    print(sd_y)
    print(sd_th)
    
    """
    ------------------------
    xref = 3-0.133
    yref = 1.467
    thref = -pi
    """
    
def plot_odo_caract():
    Data = read_txt_file("../../build/caract_odo_data.txt", 4)
    Map = read_txt_file("../../build/icp1_data.txt", 2)
    Data1 = read_txt_file("../../build/caract_odo_data2.txt", 4)
    plt.title("Odometers characterization")
    plt.plot(Map[0],Map[1])
    plt.plot(Data[1],Data[2],"o")
    plt.plot(Data1[1],Data1[2],"x")
    plt.xlabel("x [m]")
    plt.ylabel("y [m]")
    plt.grid()

    #plt.savefig("mp_data2.pdf", format="pdf")
    plt.show()
    
    t1 = [[],[],[]]
    t2 = [[],[],[]]
    t3 = [[],[],[]]
    l1 = [[],[],[]]
    l2 = [[],[],[]]
    l3 = [[],[],[]]
    size = min(len(Data[0]),len(Data1[0]))

    for i in range (size):
        if(i%3==0):
            t1[0].append(Data[1][i])
            t1[1].append(Data[2][i])
            t1[2].append(Data[3][i])
        elif (i%3==1):
            t2[0].append(Data[1][i])
            t2[1].append(Data[2][i])
            t2[2].append(Data[3][i])
        else:
            t3[0].append(Data[1][i])
            t3[1].append(Data[2][i])
            t3[2].append(Data[3][i])
            
    for i in range (size):
        if(i%3==0):
            l1[0].append(Data1[1][i])
            l1[1].append(Data1[2][i])
            l1[2].append(Data1[3][i])

        elif (i%3==1):
            l2[0].append(Data1[1][i])
            l2[1].append(Data1[2][i])
            l2[2].append(Data1[3][i])

        else:
            l3[0].append(Data1[1][i])
            l3[1].append(Data1[2][i])
            l3[2].append(Data1[3][i])

    
    plt.title("Odometers vs Lidar")
    plt.plot(t1[0],t1[1],'o',label="Odometers")
    plt.plot(l1[0],l1[1],'x',label="Lidar")
    plt.xlabel("x [m]")
    plt.ylabel("y [m]")
    plt.axis('equal')
    plt.plot(1.15,.57,'o',c='r',label="Goal")
    #plot the circle
    a = 1.15
    b = .57
    r = 0.02
    xs = np.linspace(a,a+r*(1+1/100),100)
    ys1 = np.sqrt(r**2 - (xs-a)**2)+b
    ys2 = -np.sqrt(r**2-(xs-a)**2)+b
    ys= np.array([])    
    xs = np.append(xs,xs)
    ys = np.append(ys1,ys2)
    plt.plot(xs,ys,c='g',ls=':',label="Reached goal")
    plt.plot(-(xs-a)+a,ys,c='g',ls=':')
    plt.grid()
    plt.ylim(([.54,.63]))
    plt.xlim([1.12,1.2])
    plt.legend()
    plt.savefig("goal_odo_caract.pdf", format="pdf")
    plt.show()
    
    e1 = np.array(t1)-np.array(l1)
    e2 = np.array(t2)-np.array(l2)
    e3 = np.array(t3)-np.array(l3)
    for i in range (len(e1[2])):
        if(e1[2][i]> np.pi):
            e1[2][i]-=2*np.pi
        if(e1[2][i]<-np.pi):
            e1[2][i]+=2*np.pi
        
    #plt.plot(e[0],e[1],'o')
    data1 = [e1[0],e2[0],e3[0]]
    data2 = [e1[1],e2[1],e3[1]]
    data3 = [e1[2],e2[2],e3[2]]
    #data2 = []
    #print(data)
    
    #fig,(ax1,ax2,ax3) = plt.subplots(3,sharex=True)
    
    plt.boxplot(data1)
    plt.title("Error in x")
    plt.grid()
    #plt.ylim([-0.05,0.05])
    plt.ylabel("x [m]")
    plt.show()
    
    plt.boxplot(data2)
    plt.grid()
    plt.title("Error in y")
    plt.ylim([-0.05,0.05])
    plt.ylabel("y [m]")
    plt.show()
        
    plt.boxplot(data3)
    plt.title(r"Error in $\theta$")
    plt.grid()
    plt.ylim([-.025,.11])
    plt.ylabel('angle [radian]')
    #plt.set(ylim=[-.2,.2])
    plt.show()
    
    d1 = np.sqrt(e1[0]**2+e1[1]**2)
    d2 = np.sqrt(e2[0]**2+e2[1]**2)
    d3 = np.sqrt(e3[0]**2+e3[1]**2)
    #plt.plot(d3)
    #plt.show()
    d1 = np.append(d1,d2)
    d1 = np.append(d1,d3)
    #plt.boxplot([d1,d2,d3])
    
    
    fig,(ax1,ax2) = plt.subplots(2)
    ax1.set(title="Cumulated error")
    ax1.set(ylabel="Error [m]")
    ax1.boxplot(d1)
    ax1.set(ylim=[0.0,0.06])
    #ax1.set(ylim=[0,0.05])
    ax1.grid()
    
    ax2.hist(d1,30)
    ax2.set(xlim=[0.0,0.06])
    #ax2.set(xlim=[0,0.07])
    ax2.set(xlabel="Error [m]")
    ax2.grid()
    fig.show()
    
def plot_odo_caract2():
    Data = read_txt_file("../../build/caract_odo_data.txt", 4)
    Map = read_txt_file("../../build/icp1_data.txt", 2)
    Data1 = read_txt_file("../../build/caract_odo_data2.txt", 4)
    plt.title("Odometers caracterization")
    plt.plot(Map[0],Map[1])
    print(Data)
    plt.plot(Data[1],Data[2],"o")
    plt.plot(Data1[1],Data1[2],"x")
    plt.xlabel("x [m]")
    plt.ylabel("y [m]")
    plt.grid()
    #plt.xlim([0.4,0.6])
    plt.show()

"""
plot_rpl_data()
plot_op_data()
plot_rec_data()
plot_mp_data()

plot_mp_data()
plot_op_data()
plot_icp_data()
plot_rec_data()

#plot_rpl_corr()
plot_llc_data()
plot_mp_data()
"""
#plot_icp_data()
#plot_mlc_opti()
#plot_mp_data()
#plot_llc_data()
#plot_lpf_data()
#open_loop()
#plot_lidar_caract()
#plot_mlc_opti()
#plot_odo_caract2()
#plot_odo_caract()
#plot_mp_data()

"""
th_mp =  -140.47653170302596
th_rec =  -136.28735078392694

th_mp =  -141.99836490267293
th_rec =  -141.8475624109945

th_mp =  -171.7061692844266
th_rec =  -171.17818867621355

bug sur ligne
th_mp =  79.89443816441177
th_rec =  86.07986133752608
"""




