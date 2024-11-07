 ### MAIN SCRIPT TO RUN DELTA ROBOT AND DXL STAGE FOR TRAINING TRAJECTORY ###

# import
import os
import numpy as np
import socket
import serial
import time
import csv
from datetime import datetime as dt
from dxl_comms import *


# log save name
save_name = "E9"

# trajectory filename
trajectory_filename = 'trajectories/spherical_newgantry_hysteresis_v3.csv'
# traj_speed = 25.0
# traj_z_adjust = -1.0 # in mm # NOT USED YET

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

#TODO: add this in
#x, y, z, offsets in mm
# x_offset = -1
# y_offset = 0
# z_offset = 0

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
        
        #change drive mode
        for i in range(len(dxl_ids)):
            dxl_comm_result, dxl_error = self.packetHandler.write1ByteTxRx(self.portHandler,i, ADDR_DRIVE_MODE, DRIVE_MODE)

        #Add velocity profile
                
        for i in range(len(dxl_ids)):
            param_profile_velocity = [DXL_LOBYTE(DXL_LOWORD(PROFILE_VELOCITY)), DXL_HIBYTE(DXL_LOWORD(PROFILE_VELOCITY)), DXL_LOBYTE(DXL_HIWORD(PROFILE_VELOCITY)), DXL_HIBYTE(DXL_HIWORD(PROFILE_VELOCITY))]
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
            
        # create the CSV file for the LOG
        self.traj = []
        self.save_data = []
        cur_time = str(dt.now())
        cur_time = cur_time.replace(":","_")
        self.filename = "raw_data/log_"+cur_time+"_"+save_name+".txt"
        self.log_file = open(self.filename, 'w')
        # TODO: add header to csv file here!

    '''convert from position value to dxl pulse counts **X**'''    
    def position_to_pulses_x(self, position):
        max_counts = 4095
        return round(position * (max_counts/(np.pi*self.pitch_d))) + 1997
    
    '''convert from position value to dxl pulse counts **Y1**'''    
    def position_to_pulses_y1(self, position):
        max_counts = 4095
        return 2098 - round(position * (max_counts/(np.pi*self.pitch_d)))
    
    '''convert from position value to dxl pulse counts **Y2**'''    
    def position_to_pulses_y2(self, position):
        max_counts = 4095
        return round(position * (max_counts/(np.pi*self.pitch_d))) + 1992
    
    '''convert from position value to dxl pulse counts **Z**'''    
    def position_to_pulses_z(self, position):
        max_counts = 4095
        return round(position * (max_counts/(np.pi*self.pitch_d))) + 1740 #set z offset to be such that 0 is where the sensor touches the pedestal
    

    '''convert from pulse counts to position values **X** '''    
    def pulses_to_position_x(self, counts):
        max_counts = 4095
        return (counts-1997) * (np.pi*self.pitch_d)/max_counts
    
    '''convert from pulse counts to position values **Y1**'''    
    def pulses_to_position_y1(self, counts):
        max_counts = 4095
        return (2098-counts) * (np.pi*self.pitch_d)/max_counts
    
    '''convert from pulse counts to position values **Y2**'''    
    def pulses_to_position_y2(self, counts):
        max_counts = 4095
        return (counts-1992) * (np.pi*self.pitch_d)/max_counts
    
    '''convert from pulse counts to position values **Z**'''    
    def pulses_to_position_z(self, counts):
        max_counts = 4095
        return (counts-1740) * (np.pi*self.pitch_d)/max_counts

    '''add a trajectory point'''
    def add_point(self, traj_data, save_data):
        # calculate dynamixel positions
        #traj data = x, y, z, pitch, roll
        commands_x = self.position_to_pulses_x(float(traj_data[0]))
        commands_y1 = self.position_to_pulses_y1(float(traj_data[1]))
        commands_y2 = self.position_to_pulses_y2(float(traj_data[1]))
        commands_z = self.position_to_pulses_z(float(traj_data[2]))
 
        
        commands_pitch = r2p(float(traj_data[3]))
        commands_roll = r2p(float(traj_data[4]))

        #check to make sure all commands are within gantry lims
        if (commands_y1 < y1_lims[0] or commands_y1 > y1_lims[1]) \
            or (commands_y2 < y2_lims[0] or commands_y2 > y2_lims[1]) \
            or (commands_z < z_lims[0] or commands_z > z_lims[1]) \
            or (commands_pitch < ati_pitch_lims[0] or commands_pitch > ati_pitch_lims[1]) \
            or (commands_roll <ati_roll_lims[0] or commands_roll > ati_roll_lims[1]):

            self.err_pts.append([commands_x, commands_y1, commands_y2, commands_z, commands_pitch, commands_roll])
            
        commands = [commands_x, commands_y1, commands_y2, commands_z, commands_pitch, commands_roll]
        print(commands)

        # append commands
        self.commands.append(tuple(commands))
        # append trajectory data
        self.traj.append(traj_data) 
        # append data to save
        self.save_data.append(save_data)

    '''check for good trajectory'''
    def check_traj(self):
        if len(self.err_pts) == 0:
            print("NO BAD POINTS")
            return 1
        else:
            print("BAD POINTS")
            print(self.err_pts)
            return 0

        


    '''run the gcode trajectory so far'''
    def run_hysteresis_test(self):
        commands_completed = 0
        dxl_commands = self.commands[0]

        for i in range(len(dxl_ids)):
                param_goal_position = [DXL_LOBYTE(DXL_LOWORD(dxl_commands[i])), DXL_HIBYTE(DXL_LOWORD(dxl_commands[i])), DXL_LOBYTE(DXL_HIWORD(dxl_commands[i])), DXL_HIBYTE(DXL_HIWORD(dxl_commands[i]))]
                dxl_addparam_result = self.groupSyncWrite.addParam(dxl_ids[i], param_goal_position)

        # Syncwrite goal position
        dxl_comm_result = self.groupSyncWrite.txPacket()
        time.sleep(self.dxl_delay)
        beforepos_time = time.time()

        # Clear syncwrite parameter storage
        self.groupSyncWrite.clearParam()
        
        while (commands_completed+1) < len(self.commands):

            #frequency
            # time.sleep(0.001)
            start = time.time()
            for i in dxl_ids:
                dxl_addparam_result = self.groupSyncRead.addParam(i)

            # Syncread present position
            dxl_comm_result = self.groupSyncRead.txRxPacket()

            for i in range(len(dxl_ids)):
                self.present_pos[i] = self.groupSyncRead.getData(dxl_ids[i], ADDR_PRESENT_POSITION, LEN_PRESENT_POSITION)
                
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
            # append other data we want to save
            flt_data.append(float(self.traj[commands_completed][0])) # desired x (mm)
            flt_data.append(float(self.traj[commands_completed][1])) # desired y (mm)
            flt_data.append(float(self.traj[commands_completed][2])) # desired z (mm)
            flt_data.append(float(self.save_data[commands_completed][0])) # contact flag
            # TODO: check all of this!
            flt_data.append(float(self.traj[commands_completed][3])) # desired ATI pitch (rads)
            flt_data.append(float(self.traj[commands_completed][4])) # desired ATI roll (rads)
            flt_data.append(self.present_pos[0]) # actual x (counts)
            flt_data.append(self.present_pos[1]) # actual y1 (counts)   might only need to send one of the y values - they should be the same.
            flt_data.append(self.present_pos[2]) # actual y2 (counts)
            flt_data.append(self.present_pos[3]) # actual z (counts)
            flt_data.append(self.present_pos[4]) # actual ATI pitch (counts)
            flt_data.append(self.present_pos[5]) # actual ATI roll (counts)
            flt_data.append(self.pulses_to_position_x(self.present_pos[0])) # actual x (mm)
            flt_data.append(self.pulses_to_position_y1(self.present_pos[1])) # actual y1 (mm)   might only need to send one of the y values - they should be the same.
            flt_data.append(self.pulses_to_position_y2(self.present_pos[2])) # actual y2 (mm)
            flt_data.append(self.pulses_to_position_z(self.present_pos[3])) # actual z (mm)
            flt_data.append(p2r(self.present_pos[4])) # actual ATI pitch (rads)
            flt_data.append(p2r(self.present_pos[5])) # actual ATI roll (rads)
            # flt_data.append(self.commands[commands_completed]) #desired (counts)
            flt_data.append((self.commands[commands_completed])[0]) #desired x (counts)
            flt_data.append((self.commands[commands_completed])[1]) #desired y1 (counts)
            flt_data.append((self.commands[commands_completed])[2]) #desired y2 (counts)
            flt_data.append((self.commands[commands_completed])[3]) #desired z (counts)
            flt_data.append((self.commands[commands_completed])[4]) #desired theta (counts)
            flt_data.append((self.commands[commands_completed])[5]) #desired phi (counts)
            print(flt_data)
            #TODO: add current dxl positions
            # convert data for logging
            logline = str(flt_data[0])
            for i in range(1, len(flt_data)):
                logline = logline + ", " + str(flt_data[i])
            # print(logline)
            self.log_file.write(logline)
            self.log_file.write('\n')
        
            #find difference between goal and true position
            for i in range(len(dxl_ids)):
                self.position_diff[i] = abs(dxl_commands[i] - self.present_pos[i])

            # print(self.position_diff)
            # if (np.all(self.position_diff < 2) and time.time() - beforepos_time > 0.001) or time.time() - beforepos_time > 0.1:
            if np.all(self.position_diff < 5):
                commands_completed+=1
                dxl_commands = self.commands[commands_completed]
                # print(dxl_commands)
                #command new goal position all
                for i in range(len(dxl_ids)):
                    param_goal_position = [DXL_LOBYTE(DXL_LOWORD(dxl_commands[i])), DXL_HIBYTE(DXL_LOWORD(dxl_commands[i])), DXL_LOBYTE(DXL_HIWORD(dxl_commands[i])), DXL_HIBYTE(DXL_HIWORD(dxl_commands[i]))]
                    dxl_addparam_result = self.groupSyncWrite.addParam(dxl_ids[i], param_goal_position)
                # Syncwrite goal position
                dxl_comm_result = self.groupSyncWrite.txPacket()
                time.sleep(self.dxl_delay)
                self.groupSyncWrite.clearParam()
                beforepos_time = time.time()

            print(time.time()-start)

            self.groupSyncRead.clearParam()

    '''collect data on the sensor: time_duration and time_interval are in seconds'''
    def run_sensornoise_test(self, time_duration, time_interval):
            iters = round(time_duration/time_interval)
            num_data_points = 10
            counter = 0
            start_time=time.time()
            while counter < iters:
                if time.time() - start_time >time_interval:
                    start_time = time.time()   
                    counter+=1
                                
                    # record data
                    for i in range(num_data_points):
                        # time.sleep(0.1)
                        
                    
                        # request data
                        tosend = "request"
                        self.sock.sendto(tosend.encode(), (UDP_DEST, UDP_PORT))
                        
                        # recieve data from the system
                        data, addr = self.sock.recvfrom(1024) # buffer size is 1024 bytes
                        # print(i)
                        # decode data
                        data = str(data.decode())
                        # first, split data at '\n' char
                        data = data.split("\n")
                        # then, split data at commas
                        str_data = data[0]
                        flt_data = str_data.split(",")
                        # print(str_data)
                        # print([flt_data[0:12]])
                        # convert data to floats
                        for i in range(len(flt_data)):
                            flt_data[i] = float(flt_data[i])
                        # append other data we want to save
                        
                        print(flt_data[0])
                        print(flt_data[4:12])
                        #TODO: add current dxl positions
                        # convert data for logging
                        logline = str(flt_data[0])
                        for i in range(4, 12):
                            logline = logline + ", " + str(flt_data[i])
                        # print(logline)
                        self.log_file.write(logline)
                        self.log_file.write('\n')

    '''shutdown robot'''
    def shutdown(self):
        # close the dynamixel port
        self.portHandler.closePort()
        # close log file
        self.log_file.close()



### main function ###
if __name__ == "__main__":
    # instantiate robot controller
    robot = TrainingRobotController()
    
    # load trajectory file
    try:
        with open(trajectory_filename) as csvfile:
            # next(csvfile)
            trajectory = csv.reader(csvfile)
            
            for row in trajectory:
                save_data = [row[3], row[7], row[8] ] # TODO: check this!!
                # row[12] is pitch (roty), row[13] is roll (rotx)
                traj_data = [row[0],row[1],row[2],row[12],row[13]]
                robot.add_point(traj_data, save_data)
         
    except FileNotFoundError:
        print("File not found.")

    # # testing single point
    # traj_data = [0, 0, -50, 0, 0] # sensor height is -114mm
    # save_data = [0, 0, 0]
    # robot.add_point(traj_data, save_data, speed=traj_speed)

    #check points
    # if robot.check_traj():
    #     # run the trajectory all at once
    #     print("Starting trajectory.")
    #     robot.run_hysteresis_test()

    #     # shut down 
    #     print("Done with trajectory, shutting down.")
    #     robot.shutdown()

    # robot.run_sensornoise_test(500,1)
    # robot.shutdown()
        

    #check points
    if robot.check_traj():
        # run the trajectory all at once
        print("Starting trajectory.")
        robot.run_hysteresis_test()

        # shut down 
        print("Done with trajectory, shutting down.")
        robot.shutdown()