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

    Data = read_txt_file("../../build/mlc_data.txt", 13)

    plt.plot(Data[3], Data[4], label="Measured rajectory")
    plt.plot(0,0,'go', label="Initial position")
    plt.plot(Data[1], Data[2], 'ro', label="Goal position")
    plt.title("Trajectory")
    plt.xlabel("x [m]")
    plt.ylabel("y [m]")
    plt.legend()
    plt.axis("equal")
    plt.grid()
    plt.show()

    plt.plot(Data[0], Data[5], label="d_ref")
    plt.plot(Data[0], Data[7], label="d_mes")
    plt.xlabel("t [s]")
    plt.ylabel("d [m]")
    plt.legend()
    plt.grid()
    plt.show()

    plt.plot(Data[0], Data[6], label="th_ref")
    plt.plot(Data[0], Data[8], label="th_mes")
    plt.xlabel("t [s]")
    plt.ylabel("th [rad]")
    plt.legend()
    plt.grid()
    plt.show()

    plt.plot(Data[0], Data[9], label="r_sp_ref")
    plt.plot(Data[0], Data[10], label="l_sp_ref")
    plt.xlabel("t [s]")
    plt.ylabel("w [rad]")
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

    plt.plot(Data[0], Data[10], label="ref")
    plt.plot(Data[0], Data[12], label="mes")
    plt.xlabel("t [s]")
    plt.ylabel("w [rad/s]")
    plt.title("L speed")
    plt.legend()
    plt.grid()
    plt.show()



def plot_llc_data():

    Data = read_txt_file("../../build/llc_data.txt", 7)
    
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

    plt.plot(Data[0], Data[2], label=r"$\omega_{ref}$", linewidth=2)
    plt.plot(Data[0], Data[4], label=r"$\omega_{mes,l}$", linewidth=1.2)
    plt.plot(Data[0], Data[3], label=r"$\omega_{mes,r}$", linewidth=1.2)
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
    """
    plt.plot(Data[0], Data[0], label="P")
    plt.plot(Data[0], Data[1], label="I")
    plt.plot(Data[0], Data[2], label="kphi")
    plt.xlabel("t [s]")
    plt.ylabel("cmd")
    plt.title("cmd")
    plt.legend()
    plt.grid()
    plt.show()
    
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


def plot_mlc_opti():

    Data = read_txt_file("../../build/mlc_data.txt", 13)

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
    
    
    x = []
    y = []
    for i in range(len(Data[0])):
        x.append(Data[1][i] * np.cos(Data[0][i] + pi))
        y.append(Data[1][i] * np.sin(Data[0][i] + pi))
        
    plt.plot(x, y, 'ro', label="rpl_data")
    plt.xlabel("x [m]")
    plt.ylabel("y [m]")
    plt.title("rpl")
    #plt.ylim(-0.1, 0.6)
    plt.axis("equal")
    plt.legend()
    plt.grid()
    #plt.savefig("mlcPF_pos.pdf", format="pdf")
    plt.show()
    
def plot_op_data():

    Data = read_txt_file("../../build/op_data.txt", 2)

    map_x = [0,3,3,0,0]
    map_y = [0,0,2,2,0]
    
    pt_x = []
    pt_y = []
    for i in range(len(Data[0])):
        pt_or = -Data[0][i]
        pt_x.append(1.5 + Data[1][i] * np.cos(pt_or))
        pt_y.append(0.5 + Data[1][i] * np.sin(pt_or))

    plt.plot(map_x, map_y, label="Map")
    plt.plot(Data[0], Data[1], 'ro', label="op_data")
    plt.xlabel("x [m]")
    plt.ylabel("y [m]")
    plt.title("op")
    #plt.ylim(-0.1, 0.6)
    plt.axis("equal")
    plt.legend()
    plt.grid()
    #plt.savefig("mlcPF_pos.pdf", format="pdf")
    plt.show()
    

    Data = read_txt_file("../../build/op_data.txt", 144*2+2)


#plot_rpl_data()
#plot_op_data()
plot_llc_data()