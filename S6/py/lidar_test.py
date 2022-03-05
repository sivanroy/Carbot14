from rplidar import RPLidar

lidar = RPLidar('/dev/ttyUSB0')

def lidar_data():
    try:

        #lidar.start_motor()
        info = lidar.get_info()
        print("info : ", info)

        health = lidar.get_health()
        print("health : ", health)
        for i, scan in enumerate(lidar.iter_scans()):
            print('%d: Got %d measurments' % (i, len(scan)))
            if i > 10:
                break
        lidar.stop()
        #lidar.stop_motor()
        lidar.disconnect()
    except RPLidarException:
        lidar.stop()
        lidar.disconnect()
        lidar_data()

lidar_data()

lidar.stop()
#lidar.stop_motor()
lidar.disconnect()