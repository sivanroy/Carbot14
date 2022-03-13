from rplidar import RPLidar
from time import sleep
import math

import matplotlib.pyplot as plt


#begin lidars
lidar = RPLidar('/dev/ttyUSB0')
lidar.start_motor()
info = lidar.get_info()
print(info)

theta = []; dist = []

for i, lidar_scan in enumerate(lidar.iter_scans()):
    #set scan data
    for scan in (lidar_scan):  # each scan = [quality,angle,dist]
        if i == 3:
            theta.append(scan[1]*2*math.pi/360)
            dist.append(scan[2])
            #print(scan[1], scan[2])

    if i == 3:
        break

print("len(data) = ", len(dist))
x = []; y = []
for i in range(len(dist)):
    x.append(dist[i] * math.cos(theta[i]))
    y.append(dist[i] * math.sin(theta[i]))

map_x = [530/2, -530/2, -530/2, 530/2, 530/2]
map_y = [450/2, 450/2, -450/2, -450/2, 450/2]
plt.plot(x, y, 'ro', label="Lidar data")
plt.plot(map_x, map_y, label = "Map", linewidth=2)
plt.xlabel("x [mm]")
plt.ylabel("y [mm]")
plt.title("Lidar data")
plt.grid()
plt.legend()
plt.savefig("lidar_py.pdf", format="pdf")
plt.show()

lidar.stop()
lidar.stop_motor()
lidar.disconnect()

#minibot.plot_w()