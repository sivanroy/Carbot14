import serial
if __name__ == '__main__':
    ser = serial.Serial('/dev/ttyACM0', 57600, timeout=1)
    ser.reset_input_buffer()
    while True:
        line = ser.readline().decode('utf-8').rstrip()
        print(line)
        if line == "Switch is ON":
            ser.reset_input_buffer()
            ser.write(b"Activate dynamixel")
            line = ser.readline().decode('utf-8').rstrip()
            print(line)
            ser.reset_input_buffer()
