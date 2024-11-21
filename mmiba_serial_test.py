import serial

def serial_reader():
    ser = serial.Serial(
        port='/dev/tty.usbmodem21303',
        baudrate=921600,
        timeout=1
    )

    if ser.isOpen():
        print(f"Serial port {ser.port} is open.")
    else:
        print(f"Failed to open serial port {ser.port}.")
        exit()

    while True:
        float_data = [0.0]*36
        data = ser.readline()
        try:
            split_data = data.decode().strip().split(",")
            if len(split_data) != 37:
                print("Bad data, wrong length.")
                print(len(split_data))
                ser.flush()
            else:
                for d in range(len(split_data)-1):
                    float_data[d] = float(split_data[d])
        except:
            print("Bad data, couldn't decode.")
            ser.flush()
        print(float_data)


if __name__ == "__main__":
    serial_reader()