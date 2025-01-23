# %% imports
import os
import numpy as np
import socket
import serial
import time
import csv
from datetime import datetime as dt
from dxl_comms import *
import dxl_comms
#connect to ATI

# set up ethernet for logging
UDP_IP = "192.168.1.201"  #IP of this PC (make sure ethernet settings are set to this ip)
UDP_DEST = "192.168.1.1" #IP of Nucleo Board
UDP_PORT = 11223


# set up serial to read output from training board
baud_rate = 9600


# set up for dynamixels
dxl_ids =  (1, 2, 3, 4, 5, 6)
r2p = lambda rad: round(rad * 2048 / np.pi) + 2048   # radians to pulse counts
p2r = lambda pulse: (pulse - 2048) * np.pi / 2048    # pulse counts to radians

# class for robot controller
class TrainingRobotController:
    '''Instantiates the Robot Class w/ port'''
    def __init__(self):
        
        # create ethernet socket
        print("Creating ethernet socket for sensor sampling.")
        self.sock = socket.socket(socket.AF_INET, # Internet
                     socket.SOCK_DGRAM) # UDP
        self.sock.bind((UDP_IP, UDP_PORT))
        
        # create comms for dynamixels
        self.portHandler, self.packetHandler, self.groupSyncWrite, self.groupSyncRead, self.groupSyncWrite_PROF_VEL = initComms()
        self.dxl_delay = 0.01 # wait time between sending and reading

        # Disable Dynamixel Torques
        for i in dxl_ids:
            dxl_comm_result, dxl_error = self.packetHandler.write1ByteTxRx(self.portHandler,i, ADDR_TORQUE_ENABLE, TORQUE_DISABLE)
            if dxl_comm_result != COMM_SUCCESS:
                print("%s" % self.packetHandler.getTxRxResult(dxl_comm_result))
            elif dxl_error != 0:
                print("%s" % self.packetHandler.getRxPacketError(dxl_error))
            else:
                print("Dynamixel#%d has been successfully connected" %i)
        
        #Add velocity profile
        for i in range(len(dxl_ids)):
            param_profile_velocity = [DXL_LOBYTE(DXL_LOWORD(PROFILE_VELOCITY_CALIBRATE)), DXL_HIBYTE(DXL_LOWORD(PROFILE_VELOCITY_CALIBRATE)), DXL_LOBYTE(DXL_HIWORD(PROFILE_VELOCITY_CALIBRATE)), DXL_HIBYTE(DXL_HIWORD(PROFILE_VELOCITY_CALIBRATE))]
            dxl_addparam_result = self.groupSyncWrite_PROF_VEL.addParam(dxl_ids[i], param_profile_velocity)

        dxl_comm_result = self.groupSyncWrite_PROF_VEL.txPacket()
        print("%s" % self.packetHandler.getTxRxResult(dxl_comm_result))
        time.sleep(self.dxl_delay)
        # Clear syncwrite parameter storage
        self.groupSyncWrite.clearParam()

        # Enable Dynamixel Torques
        for i in dxl_ids:
            dxl_comm_result, dxl_error = self.packetHandler.write1ByteTxRx(self.portHandler,i, ADDR_TORQUE_ENABLE, TORQUE_ENABLE)

        #Initialize 0,0 position for force sensor
        self.ati_zero = 2048
        ati_ids = [5, 6]  
        for i in range(len(ati_ids)):
            param_goal_position = [DXL_LOBYTE(DXL_LOWORD(self.ati_zero)), DXL_HIBYTE(DXL_LOWORD(self.ati_zero)), DXL_LOBYTE(DXL_HIWORD(self.ati_zero)), DXL_HIBYTE(DXL_HIWORD(self.ati_zero))]
            dxl_addparam_result = self.groupSyncWrite.addParam(ati_ids[i], param_goal_position)
        
        # Syncwrite goal position
        dxl_comm_result = self.groupSyncWrite.txPacket()
        time.sleep(self.dxl_delay)

        # Clear syncwrite parameter storage
        self.groupSyncWrite.clearParam()
        time.sleep(1) #wait for dxl to get to their positions
        
        # setup initial commands to delta
        # commands are tuples of gcode string and list of dynamixel positions
        self.commands = []
        self.present_pos = [0, 0, 0, 0, 0, 0]
        self.moving_status = np.array([0, 0, 0, 0, 0, 0])
        self.pitch_d = 28.01 #mm
        # TODO: add an initial set of dynamixel commands to self.commands if needed 
        #TODO: set max speed
        self.err_pts = []
        self.position_diff = np.array([0,0,0,0,0,0])
        
    
        # TODO: add header to csv file here!
        '''convert from position value to dxl pulse counts **X**'''    
    def position_to_pulses_x(self, position):
        return round(position * (MAX_COUNTS/(np.pi*PITCH_D))) + 2048

    '''convert from position value to dxl pulse counts **Y1**'''    
    def position_to_pulses_y1(self, position):
        return 2048 - round(position * (MAX_COUNTS/(np.pi*PITCH_D)))

    '''convert from position value to dxl pulse counts **Y2**'''    
    def position_to_pulses_y2(self, position):
        return round(position * (MAX_COUNTS/(np.pi*PITCH_D))) + 2048

    '''convert from position value to dxl pulse counts **Z**'''    
    def position_to_pulses_z(self, position):
        return round(position * (MAX_COUNTS/(np.pi*PITCH_D))) + 2048 #set z offset to be such that 0 is where the sensor touches the pedestal

    '''convert from pulse counts to position values **X** '''    
    def pulses_to_position_x(self, counts):
        return (counts-2048) * (np.pi*PITCH_D)/MAX_COUNTS

    '''convert from pulse counts to position values **Y1**'''    
    def pulses_to_position_y1(self, counts):
        return (2048-counts) * (np.pi*PITCH_D)/MAX_COUNTS

    '''convert from pulse counts to position values **Y2**'''    
    def pulses_to_position_y2(self, counts):
        return (counts-2048) * (np.pi*PITCH_D)/MAX_COUNTS

    '''convert from pulse counts to position values **Z**'''    
    def pulses_to_position_z(self, counts):
        return (counts-2048) * (np.pi*PITCH_D)/MAX_COUNTS

    def run_z_calibration(self):
        #TODOs:
        # only need to read and write to z value
        z_dxl_ind = 3
        z_home_coord = 0 # mm
        z_approach_coord = -20 # mm
        y_approach_coord = 10.68 #for ellipsoid: 10.68 #mm

        z_home_coord_counts = self.position_to_pulses_z(z_home_coord)
        z_approach_coord_counts = self.position_to_pulses_z(z_approach_coord)
        y1_approach_coord_counts = self.position_to_pulses_y1(y_approach_coord)
        y2_approach_coord_counts = self.position_to_pulses_y2(y_approach_coord)

        home_coordinate = [2048, y1_approach_coord_counts, y2_approach_coord_counts, z_home_coord_counts, 2048, 2048]
        approach_coordinate = [2048, y1_approach_coord_counts, y2_approach_coord_counts, z_approach_coord_counts, 2048, 2048]

        print("home coordinate: ", home_coordinate)
        print("approach coordinate: ", approach_coordinate)

        
        #check to make sure all commands are within Z gantry lims
        if z_home_coord_counts < z_lims[0] or z_home_coord_counts > z_lims[1]:
            print("home coordinate is out of bounds")
            return
        
        elif z_approach_coord_counts < z_lims[0] or z_approach_coord_counts > z_lims[1]:
            print("approach coordinate is out of bounds")
            return

        prev_ati_value = 0
        diff_thresh = 0.5 #N

        # go to home coordinate
        for i in range(len(dxl_ids)):
            param_goal_position = [DXL_LOBYTE(DXL_LOWORD(home_coordinate[i])), DXL_HIBYTE(DXL_LOWORD(home_coordinate[i])), DXL_LOBYTE(DXL_HIWORD(home_coordinate[i])), DXL_HIBYTE(DXL_HIWORD(home_coordinate[i]))]
            dxl_addparam_result = self.groupSyncWrite.addParam(dxl_ids[i], param_goal_position)
        # Syncwrite goal position
        dxl_comm_result = self.groupSyncWrite.txPacket()
        time.sleep(self.dxl_delay)
        # Clear syncwrite parameter storage
        self.groupSyncWrite.clearParam()

        time.sleep(4)

        
        
        # go to approach coordinate
        for i in range(len(dxl_ids)):
            param_goal_position = [DXL_LOBYTE(DXL_LOWORD(approach_coordinate[i])), DXL_HIBYTE(DXL_LOWORD(approach_coordinate[i])), DXL_LOBYTE(DXL_HIWORD(approach_coordinate[i])), DXL_HIBYTE(DXL_HIWORD(approach_coordinate[i]))]
            dxl_addparam_result = self.groupSyncWrite.addParam(dxl_ids[i], param_goal_position)
        # Syncwrite goal position
        dxl_comm_result = self.groupSyncWrite.txPacket()
        time.sleep(self.dxl_delay)
        # Clear syncwrite parameter storage
        self.groupSyncWrite.clearParam()


        while 1:
            # Syncread present position

            dxl_addparam_result = self.groupSyncRead.addParam(dxl_ids[z_dxl_ind])
            dxl_comm_result = self.groupSyncRead.txRxPacket()
            self.present_pos[z_dxl_ind] = self.groupSyncRead.getData(dxl_ids[z_dxl_ind], ADDR_PRESENT_POSITION, LEN_PRESENT_POSITION)

            # request data
            tosend = "request"
            self.sock.sendto(tosend.encode(), (UDP_DEST, UDP_PORT))
            # receive data from the system
            data, addr = self.sock.recvfrom(1024) # buffer size is 1024 bytes
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

            #if contact is detected, break the loop
            if abs(flt_data[3] - prev_ati_value) > diff_thresh:
                print("detected contact")
                print("offset count: ", self.present_pos[z_dxl_ind])
                break

            #if we've reached the final position, break the loop
            elif (self.present_pos[z_dxl_ind] - z_approach_coord_counts) <= 3:
                print("z_approach_coordinate counts: ", z_approach_coord_counts)
                print("reached final coordinate, lifting up")
                break
        
        lift_amount = 50 #counts
        backup_z_coordinate = self.present_pos[z_dxl_ind] + lift_amount #******* make sure this is within bounds

        # go to backup coordinate

        param_goal_position = [DXL_LOBYTE(DXL_LOWORD(backup_z_coordinate)), DXL_HIBYTE(DXL_LOWORD(backup_z_coordinate)), DXL_LOBYTE(DXL_HIWORD(backup_z_coordinate)), DXL_HIBYTE(DXL_HIWORD(backup_z_coordinate))]
        dxl_addparam_result = self.groupSyncWrite.addParam(dxl_ids[z_dxl_ind], param_goal_position)
        # Syncwrite goal position
        dxl_comm_result = self.groupSyncWrite.txPacket()
        time.sleep(self.dxl_delay)
        # Clear syncwrite parameter storage
        self.groupSyncWrite.clearParam()
        time.sleep(1)
        # Raise velocity limit 


        #go back to home coordinate
        for i in range(len(dxl_ids)):
            param_goal_position = [DXL_LOBYTE(DXL_LOWORD(home_coordinate[i])), DXL_HIBYTE(DXL_LOWORD(home_coordinate[i])), DXL_LOBYTE(DXL_HIWORD(home_coordinate[i])), DXL_HIBYTE(DXL_HIWORD(home_coordinate[i]))]
            dxl_addparam_result = self.groupSyncWrite.addParam(dxl_ids[i], param_goal_position)
        # Syncwrite goal position
        dxl_comm_result = self.groupSyncWrite.txPacket()
        time.sleep(self.dxl_delay)
        # Clear syncwrite parameter storage
        self.groupSyncWrite.clearParam()
        time.sleep(2)


    '''shutdown robot'''
    def shutdown(self):
        # close the dynamixel port
        self.portHandler.closePort()
        


### main function ###
if __name__ == "__main__":
    # instantiate robot controller
    robot = TrainingRobotController()

    #check points
    robot.run_z_calibration()

    # shut down 
    print("Done with trajectory, shutting down.")
    robot.shutdown()


    
# set dynamixels to 0 position -> z to +20 or something
# lower down z slowly until ATI reading changes by some threshold. (do i also need to filter)

# print this value out as an offset from z = 0 in mm and in counts

#go back to home/ clearance height. 