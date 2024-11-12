# IMPORTS
import os
import numpy as np
import socket
import serial
import time

# SETUP

# set up ethernet for logging
UDP_IP = "192.168.1.201"  #IP of this PC (make sure ethernet settings are set to this ip)
UDP_DEST = "192.168.1.1" #IP of Nucleo Board
UDP_PORT = 11223
# set up serial to read output from training board
baud_rate = 9600

# create ethernet socket
print("Creating ethernet socket for sensor sampling.")
my_sock = socket.socket(socket.AF_INET, # Internet
                socket.SOCK_DGRAM) # UD
my_sock.bind((UDP_IP, UDP_PORT))


# TEST
while True:

    # wait a bit 
    time.sleep(0.01)

    # request data
    tosend = "request"
    my_sock.sendto(tosend.encode(), (UDP_DEST, UDP_PORT))

    # recieve data from the system
    data, addr = my_sock.recvfrom(1024) # buffer size is 1024 bytes
    # decode data
    data = str(data.decode())
    # first, split data at '\n' char
    data = data.split("\n")
    # then, split data at commas
    str_data = data[0]
    flt_data = str_data.split(",")
    # convert data to floats
    for i in range(len(flt_data)):
        flt_data[i] = float(flt_data[i])
    # print it out
    print("\033c", end="")
    print("%5.2f, % 3.2f, % 3.2f, % 3.2f" % (flt_data[0], flt_data[1], flt_data[2], flt_data[3]))
    # print(flt_data[0:4])