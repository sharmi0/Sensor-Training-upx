# Author: elchun
# TODO: Get dxl data asyrchonously
# Maybe we can get the dxl data at 50 Hz and 
# repeat it for the final one.
# just have to make sure time stamps are synched or smth like that
# One way to do this is to 
#   1. Write dxl data to a buffer that is shared between threads
#   2. Add lock to buffer
#   3. Sensor only consumes when it needs and uses that to write.

# Another option is:
#   1. Store dxl data with timestamps (to a buffer)
#   2. store sensor data with timestamps to another buffer
#   3. run trial
#   4. merge the data by setting the dxl position reading to the most recent
#      result of the dxl get for each value in the sensor.
#       - Might be able to do this with numpy bin.  This is probably the better approach

# TODO: Zero when we move to a new location

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
from dataclasses import dataclass
import tqdm
import math

import multiprocessing
from multiprocessing import Value, Queue


# %% Config
@dataclass
class Config:
    log_save_name: str
    trajectory_filename: str

    UDP_IP: str  # IP of the computer running the script
    UDP_DEST: str  # IP of Nucleo Board
    UDP_PORT: int 

    baud_rate: str




# %% Create Controller


class TrainingRobotController:
    """
    Instantiates the Robot Controller with port
    """

    dxl_ids = (1, 2, 3, 4, 6, 5)  # Actual ids are switched from the gantry labels

    r2p = lambda rad: round(rad * 2048 / np.pi) + 2048  # radians to pulse counts
    p2r = lambda pulse: (pulse - 2048) * np.pi / 2048  # pulse counts to radians

    x_lims = dxl_comms.x_lims
    y1_lims = dxl_comms.y1_lims
    y2_lims = dxl_comms.y2_lims
    z_lims = dxl_comms.z_lims
    # ati_theta_lims = dxl_comms.ati_pitch_lims
    # ati_phi_lims = dxl_comms.ati_roll_lims

    ati_theta_lims = dxl_comms.ati_roll_lims
    ati_phi_lims = dxl_comms.ati_pitch_lims

    data_key = {
        "time": 0,

        "Fx": 1,
        "Fy": 2,
        "Fz": 3,

        "s1": 4,
        "s2": 5,
        "s3": 6,
        "s4": 7,
        "s5": 8,
        "s6": 9,
        "s7": 10,
        "s8": 11,

        "tof1": 12,
        "tof2": 13,
        "tof3": 14,
        "tof4": 15,

        "x_act": 16,
        "y1_act": 17,
        "y2_act": 18,
        "z_act": 19,

        "theta_act": 20,  # theta
        "phi_act": 21,  # phi

        "data_valid": 22,

    }

    config = None

    def __init__(self, config):
        self.config = config

        self.setup_ethernet()
        self.setup_dxl()
        self.setup_icl_sensor()
        self.setup_commands()
        self.setup_log()

    @staticmethod
    def r2p(rad):
        return round(rad * 2048 / np.pi) + 2048  # radians to pulse counts

    @staticmethod
    def p2r(pulse):
        return(pulse - 2048) * np.pi / 2048  # pulse counts to radians


    def setup_ethernet(self):
        print("Creating ethernet socket for sensor sampling.")
        self.sock = socket.socket(
            socket.AF_INET,  # Internet
            socket.SOCK_DGRAM,
        )  # UDP
        self.sock.bind((self.config.UDP_IP, self.config.UDP_PORT))

    def setup_dxl(self):
        # create comms for dynamixels
        (
            self.portHandler,
            self.packetHandler,
            self.groupSyncWrite,
            self.groupSyncRead,
            self.groupSyncWrite_PROF_VEL
        ) = initComms()
        self.dxl_delay = 0.01  # wait time between sending and reading

        # Enable Dynamixel Torques
        for i in self.dxl_ids:
            dxl_comm_result, dxl_error = self.packetHandler.write1ByteTxRx(
                self.portHandler, i, ADDR_TORQUE_ENABLE, TORQUE_ENABLE
            )
            if dxl_comm_result != COMM_SUCCESS:
                print("%s" % self.packetHandler.getTxRxResult(dxl_comm_result))
            elif dxl_error != 0:
                print("%s" % self.packetHandler.getRxPacketError(dxl_error))
            else:
                print("Dynamixel#%d has been successfully connected" % i)

        # Syncwrite goal position
        dxl_comm_result = self.groupSyncWrite.txPacket()
        time.sleep(self.dxl_delay)

        # Clear syncwrite parameter storage
        self.groupSyncWrite.clearParam()
        time.sleep(1)  # wait for dxl to get to their positions
    
    def setup_icl_sensor(self):
        usb_port = "/dev/ttyACM1"
        self.baro = serial.Serial(usb_port, 2000000, timeout=0.5)
        for _ in range(10):
            self.baro.readline()

    def setup_commands(self):
        # setup initial commands to robot
        # commands are tuples of gcode string and list of dynamixel positions
        self.commands = []
        self.present_pos = [0, 0, 0, 0, 0, 0]
        self.pitch_d = 28.01  # mm
        # TODO: add an initial set of dynamixel commands to self.commands if needed
        # TODO: set max speed
        self.err_pts = []

    def setup_log(self):
        # create the CSV file for the LOG
        self.traj = []
        self.save_data = []
        cur_time = str(dt.now())
        cur_time = cur_time.replace(":", "-")
        cur_time = cur_time.replace(" ", "_")

        self.data_dir_name = os.path.join("raw_data", cur_time + "_" + self.config.log_save_name)
        if not os.path.exists(self.data_dir_name):
            os.mkdir(self.data_dir_name)

        np.save(os.path.join(self.data_dir_name, "key.npy"), self.data_key)
        
        # self.data_filename = (
        #     "raw_data/data_" + cur_time + "_" + self.config.log_save_name + ".npy"
        # )
        # self.key_filename = (
        #     "raw_data/key_" + cur_time + "_" + self.config.log_save_name + ".npy"
        # )

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
        z_offset = 1680 # was 1740
        return round(position * (max_counts/(np.pi*self.pitch_d))) + z_offset #set z offset to be such that 0 is where the sensor touches the pedestal
    

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

    # def add_point(self, traj_data, save_data):
    #     """add a trajectory point"""
    #     # calculate dynamixel positions
    #     # traj data = x, y, z, theta, phi
    #     commands_x = self.position_to_pulses_x(float(traj_data[0]))
    #     commands_y1 = self.position_to_pulses_y1(float(traj_data[1]))
    #     commands_y2 = self.position_to_pulses_y2(float(traj_data[1]))
    #     commands_z = self.position_to_pulses_z(float(traj_data[2]))

    #     commands_theta = self.r2p(float(traj_data[3]))
    #     commands_phi = self.r2p(float(traj_data[4]))

    #     # check to make sure all commands are within gantry lims
    #     if (
    #         (commands_y1 < self.y1_lims[0] or commands_y1 > self.y1_lims[1])
    #         or (commands_y2 < self.y2_lims[0] or commands_y2 > self.y2_lims[1])
    #         or (commands_z < self.z_lims[0] or commands_z > self.z_lims[1])
    #         or (
    #             commands_theta < self.ati_theta_lims[0] or commands_theta > self.ati_theta_lims[1]
    #         )
    #         or (commands_phi < self.ati_phi_lims[0] or commands_phi > self.ati_phi_lims[1])
    #     ):
    #         self.err_pts.append(
    #             [
    #                 commands_x,
    #                 commands_y1,
    #                 commands_y2,
    #                 commands_z,
    #                 commands_phi,
    #                 commands_theta,
    #             ]
    #         )

    #     # commands = [commands_x, commands_y1, commands_y2, commands_z, commands_theta, commands_phi]
    #     # because orientation of ati sensor changed:
    #     commands = [
    #         commands_x,
    #         commands_y1,
    #         commands_y2,
    #         commands_z,
    #         commands_theta,
    #         commands_phi,
    #     ]
    #     print(commands)

    #     # append commands
    #     self.commands.append(tuple(commands))
    #     # append trajectory data
    #     self.traj.append(traj_data)
    #     # append data to save
    #     self.save_data.append(save_data)
    
    def add_point(self,traj_data): 
        """add a trajectory point"""
        # x, y, z, theta, phi, t_dwell, I_tare

        # calculate dynamixel positions
        # traj data = x, y, z, theta, phi
        commands_x = self.position_to_pulses_x(float(traj_data[0]))
        commands_y1 = self.position_to_pulses_y1(float(traj_data[1]))
        commands_y2 = self.position_to_pulses_y2(float(traj_data[1]))
        commands_z = self.position_to_pulses_z(float(traj_data[2]))

        commands_theta = self.r2p(float(traj_data[3]))
        commands_phi = self.r2p(float(traj_data[4]))

        commands_tdwell = traj_data[5]
        commands_Itare = traj_data[6]

        # check to make sure all commands are within gantry lims
        if (
            (commands_y1 < self.y1_lims[0] or commands_y1 > self.y1_lims[1])
            or (commands_y2 < self.y2_lims[0] or commands_y2 > self.y2_lims[1])
            or (commands_z < self.z_lims[0] or commands_z > self.z_lims[1])
            or (
                commands_theta < self.ati_theta_lims[0] or commands_theta > self.ati_theta_lims[1]
            )
            or (commands_phi < self.ati_phi_lims[0] or commands_phi > self.ati_phi_lims[1])
        ):
            self.err_pts.append(
                [
                    commands_x,
                    commands_y1,
                    commands_y2,
                    commands_z,
                    commands_theta,
                    commands_phi,
                ]
            )
        
        else:

            # commands = [commands_x, commands_y1, commands_y2, commands_z, commands_theta, commands_phi]
            # because orientation of ati sensor changed:
            commands = [
                commands_x,
                commands_y1,
                commands_y2,
                commands_z,
                commands_theta,
                commands_phi,
                commands_tdwell,
                commands_Itare,
            ]
            print(commands)

            # append commands
            self.commands.append(tuple(commands))
            # append trajectory data
            # self.traj.append(traj_data)
            # append data to save
    
    def load_trajectory(self, n_commands):
        trajectory = np.load(self.config.trajectory_filename)

        if n_commands is not None:
            trajectory = trajectory[:n_commands]
        for command in trajectory:
            self.add_point(command)









        # # load trajectory file
        # try:
        #     with open(self.config.trajectory_filename) as csvfile:
        #         trajectory = csv.reader(csvfile)
        #         # for row in trajectory:
        #         #     data = row[8:13] 
        #         #     robot.add_point(data, data, speed=25.0)
        #         # TODO: this is the old import code, need to figure out which set of indices are correct and make traj files match
        #         for row in trajectory:
        #             save_data = [row[3], row[7], row[8] ] # TODO: check this!!
        #             # row[12] is pitch (roty), row[13] is roll (rotx)
        #             traj_data = [row[0],row[1],row[2],row[12],row[13]]
        #             self.add_point(traj_data, save_data)
        # except FileNotFoundError:
        #     print("File not found.")

    def load_debug_trajectory(self, debug_traj):
        for point in debug_traj:
            self.add_point(point)


    def check_traj(self):
        """check for good trajectory"""
        if len(self.err_pts) == 0:
            print("NO BAD POINTS")
            return 1
        else:
            print("BAD POINTS")
            print(self.err_pts)
            return 0
    
    def get_sensor_data(self):
        tosend = "request"
        self.sock.sendto(tosend.encode(), (self.config.UDP_DEST, self.config.UDP_PORT))

        # receive data from system
        data, addr = self.sock.recvfrom(1024) # buffer size is 1024 bytes
        # decode data
        data = str(data.decode()).split('\n')[0]  # Recover first part of buffer

        str_data = data
        # if i ==0:
        # if (str_data.strip()):
        float_data = [float(d) for d in str_data.split(",")]
        return float_data
    
    def get_dxl_data(self):
        """Write dxl data to present_position"""
        #read dynamixel values: x, y, z, theta, phi
        #read present position value
        for i in self.dxl_ids:
            dxl_addparam_result = self.groupSyncRead.addParam(i)

        # Syncread present position
        dxl_comm_result = self.groupSyncRead.txRxPacket()

        for i in range(len(self.dxl_ids)):
            self.present_pos[i] = self.groupSyncRead.getData(self.dxl_ids[i], ADDR_PRESENT_POSITION, LEN_PRESENT_POSITION)

        # Clear syncwrite parameter storage
        self.groupSyncRead.clearParam()

        self.present_pos[0] = self.pulses_to_position_x(self.present_pos[0])
        self.present_pos[1] = self.pulses_to_position_y1(self.present_pos[1])
        self.present_pos[2] = self.pulses_to_position_y2(self.present_pos[2])
        self.present_pos[3] = self.pulses_to_position_z(self.present_pos[3])
        self.present_pos[4] = self.p2r(self.present_pos[4])
        self.present_pos[5] = self.p2r(self.present_pos[5])
        return self.present_pos
    
    # def get_icl_data(self):
    #     """Pull data from icl sensor over serial"""


    #     # TODO
    
    def run_trajectory_parallel(self):
        """
        run the gcode trajectory so far
        
        The idea here is that we want to sample the sensor at a set frequency (100Hz) but the time for
        dynamixels and stuff is not constant. 

        So we can sample the sensor as fast as possible then interpolate the dynamixel data to match the
        sensor data.

        https://docs.python.org/3/library/multiprocessing.html#module-multiprocessing

        Can import locks

        I can sync them to the system clock?
        """

        sensor_frequency = 100
        sensor_loop_time = 1 / sensor_frequency

        dxl_frequency = 5  # Should be a factor of sensor frequency
        dxl_loop_time = 1 / dxl_frequency

        n_sensor_samples = 20000
        n_dxl_samples = int((dxl_frequency / sensor_frequency) * n_sensor_samples)


        def sample_worker(start_time, n_samples, loop_time, sample_fn, data_queue, name, done_flag):

            # all_data = []
            overrun_count = 0

            i = 0
            while done_flag.value == 0:
                # Wait for the next sample time

                sleep_time = start_time + i * loop_time - time.perf_counter()
                if sleep_time > 0:
                    time.sleep(sleep_time)
                elif sleep_time < 0 and -sleep_time < loop_time:
                    print(f"Loop overran for {sleep_time}")
                    overrun_count += 1
                    if overrun_count / i > 0.01:  # want less than 1 percent to be overrun
                        raise ValueError("Overran too much")

                # Sample data
                sample = sample_fn()
                # sample = [time.perf_counter()]
                data_queue.put(sample)
                # all_data.append(sample)

                i += 1
            
            print(overrun_count)

        def icl_data_worker(start_time, n_samples, loop_time, data_queue, name, done_flag):
            next_i = 0

            while done_flag.value == 0:
                line = self.baro.readline()  # The first few datapoints will be slow because the buffer will be full

                if start_time + next_i * loop_time - time.perf_counter() < 0:
                    # print(self.baro.in_waiting)
                    next_i += 1
                    list_data = line.decode().strip().split(',')[:-1]
                    float_data = [float(x) for x in list_data]
                    # float_data = [float(x) for x in line.decode().split(",")]
                    # print(float_data)
                    if float_data:
                        data_queue.put(float_data)
                    # save data
            print("DONE")


        def aggregator_worker(sensor_queue, dxl_queue, icl_data_queue, n_sensor_samples, sensor_frequency, dxl_frequency, done_flag, tare_flag, data_valid_flag):

            dxl_period = int(sensor_frequency / dxl_frequency)
            sensor_data = None
            dxl_data = None

            all_data = []
            data_valid = 0 
            i = 0
            while (done_flag.value == 0):
                try:
                    sensor_data = sensor_queue.get(block=True, timeout=10)  # If it looks odd, its probably here
                    icl_data = icl_data_queue.get(block=True, timeout=10)
                    # print(icl_data)

                    if i % dxl_period == 0:
                        dxl_data = dxl_queue.get(block=True, timeout=10)
                except:
                    print("Lost data collection")
                    break

                # print("hi")
                with data_valid_flag.get_lock():
                    data_valid = data_valid_flag.value
                
                timestamp = sensor_data[0:1]
                force_data = sensor_data[1:4]
                combo_data = timestamp + force_data + icl_data + dxl_data + [data_valid]
# 
# 
                # combo_data = sensor_data + dxl_data + [data_valid] # dxl data is just the most recent one

                # print(sensor_data[4:12])
                for j in range(1, 6): # Fx Fy Fz s1 s2
                    print(f"{combo_data[j]:+.4f}", end = " ")
                print()

                # for j in range(1, 12):
                #     print(f"{sensor_data[j]:+.4f}", end = " ")
                # print(len(all_data))
                #         tare_flag.value = 0
                #         all_data.append([-1 for _ in range(len(combo_data))])
                all_data.append(combo_data)

                if (len(all_data) % 1000 == 0):
                    data_filename = os.path.join(self.data_dir_name, f"segment_{i}.npy")
                    print(data_filename)
                    np.save(data_filename, all_data)
                    all_data = []
                    # np.save(self.data_filename, all_data)
                    # np.save(self.key_filename, self.data_key)
                    print("Saved")
                i += 1
            

            data_filename = os.path.join(self.data_dir_name, f"segment_{i}.npy")
            np.save(data_filename, all_data)

            if done_flag.value == 0:
                raise Exception("QUEUE TIMEDOUT")
        
        def dxl_control_worker(done_flag, tare_flag, data_valid_flag):
            
            pbar = tqdm.tqdm(enumerate(self.commands), total=len(self.commands))
            data_valid = True
            for i, command in pbar:
                # print("\nContact %d of %d\n" % (j+1,len(self.commands)))

                # update and send dynamixel positions
                dxlx_des = (command[0])
                dxly1_des = (command[1])
                dxly2_des = (command[2])
                dxlz_des = (command[3]) 
                dxlt_des = (command[4])
                dxlp_des = (command[5])

                t_dwell = command[6]
                I_tare = command[7]

                
                dxl_commands = [dxlx_des, dxly1_des, dxly2_des, dxlz_des, dxlt_des, dxlp_des]
                # print("DXL COMMAND: ", dxl_commands)
                # print(dxl_commands)

                if i == 0 or dxlt_des != self.commands[i-1][4] or dxlp_des != self.commands[i-1][5]:
                    with data_valid_flag.get_lock():
                        data_valid_flag.value = 0
                else:
                    to_sleep = False
                    with data_valid_flag.get_lock():
                        to_sleep = data_valid_flag.value == 0
                        data_valid_flag.value = 1
                    if to_sleep:
                        time.sleep(0.2)  # Give time to tare
                
                #command new goal position all
                for i in range(len(self.dxl_ids)):
                    # print("DXL ID: ", self.dxl_ids[i], "DXL_COMMAND: ", dxl_commands[i])
                    param_goal_position = [DXL_LOBYTE(DXL_LOWORD(dxl_commands[i])), DXL_HIBYTE(DXL_LOWORD(dxl_commands[i])), DXL_LOBYTE(DXL_HIWORD(dxl_commands[i])), DXL_HIBYTE(DXL_HIWORD(dxl_commands[i]))]
                    dxl_addparam_result = self.groupSyncWrite.addParam(self.dxl_ids[i], param_goal_position)

                # Syncwrite goal position
                dxl_comm_result = self.groupSyncWrite.txPacket()
                time.sleep(self.dxl_delay)

                # Clear syncwrite parameter storage
                self.groupSyncWrite.clearParam()
                time.sleep(t_dwell) #wait for dxl to get to their positions


                # -- Check that we got to final position -- #
                # for i in self.dxl_ids:
                #     dxl_addparam_result = self.groupSyncRead.addParam(i)

                # # Syncread present position
                # dxl_comm_result = self.groupSyncRead.txRxPacket()

                # present_position = [0 for _ in range(len(self.dxl_ids))]
                # for i in range(len(self.dxl_ids)):
                #     present_position[i] = self.groupSyncRead.getData(self.dxl_ids[i], ADDR_PRESENT_POSITION, LEN_PRESENT_POSITION)
                
                # for i in range(len(dxl_commands)):
                #     if abs(present_position[i] - dxl_commands[i]) > 10:
                #         print(present_position)
                #         print(dxl_commands)
                #         print("TOO FAR")
                # print(present_position)

            

                # if dxlt_des != self.commands[i-1][4] or dxlp_des != self.commands[i-1][5]:
                #     print("Zeroing")
                #     with tare_flag.get_lock():
                #         tare_flag.value = 1
                #     time.sleep(0.1)
                # if I_tare:
                #     with tare_flag.get_lock():
                #         tare_flag.value = 1
                #     time.sleep(0.1)
            
            done_flag.value = 1


        
        def sensor_sample_function():
            data = self.get_sensor_data()
            # print(time.perf_counter(), "sen", data)
            return data
        
        def dxl_sample_function():
            data = self.get_dxl_data()
            # print(time.perf_counter(), "dxl", data)
            return data
        
        start_time = time.perf_counter() + 1.0

        dxl_queue = Queue()
        sensor_queue = Queue()
        icl_data_queue = Queue()
        
        done_flag = Value('i', 0)
        tare_flag = Value('i', 0)
        data_valid_flag = Value('i', 1)
        
        dxl_process = multiprocessing.Process(target=sample_worker, 
                args=(start_time, n_dxl_samples, dxl_loop_time, dxl_sample_function, dxl_queue, "dxl", done_flag))
        sensor_process = multiprocessing.Process(target=sample_worker, 
                args=(start_time, n_sensor_samples, sensor_loop_time, sensor_sample_function, sensor_queue, "pressure", done_flag))
        aggregator_process = multiprocessing.Process(target=aggregator_worker,
                args=(sensor_queue, dxl_queue, icl_data_queue, n_sensor_samples, sensor_frequency, dxl_frequency, done_flag, tare_flag, data_valid_flag))

        dxl_control_process = multiprocessing.Process(target=dxl_control_worker,
                                                      args=(done_flag, tare_flag, data_valid_flag))
        
        icl_sensor_process = multiprocessing.Process(target=icl_data_worker,
                                                     args=(start_time, None, sensor_loop_time, icl_data_queue, "icl", done_flag))


        # -- Start data collection -- #
        dxl_process.start()
        sensor_process.start()
        icl_sensor_process.start()
        aggregator_process.start()

        # self.commands = self.commands[:8]  # For testing

        time.sleep(1.0)
        dxl_control_process.start()

        # for j, command in enumerate(self.commands):
        #     # print("\nContact %d of %d\n" % (j+1,len(self.commands)))

        #     # update and send dynamixel positions
        #     dxlx_des = (command[0])
        #     dxly1_des = (command[1])
        #     dxly2_des = (command[2])
        #     dxlz_des = (command[3]) 
        #     dxlt_des = (command[4])
        #     dxlp_des = (command[5])

            
        #     dxl_commands = [dxlx_des, dxly1_des, dxly2_des, dxlz_des, dxlt_des, dxlp_des]
        #     # print(dxl_commands)
            
        #     #command new goal position all
        #     for i in range(len(self.dxl_ids)):
        #         param_goal_position = [DXL_LOBYTE(DXL_LOWORD(dxl_commands[i])), DXL_HIBYTE(DXL_LOWORD(dxl_commands[i])), DXL_LOBYTE(DXL_HIWORD(dxl_commands[i])), DXL_HIBYTE(DXL_HIWORD(dxl_commands[i]))]
        #         dxl_addparam_result = self.groupSyncWrite.addParam(self.dxl_ids[i], param_goal_position)

        #     # Syncwrite goal position
        #     dxl_comm_result = self.groupSyncWrite.txPacket()
        #     time.sleep(self.dxl_delay)

        #     # Clear syncwrite parameter storage
        #     self.groupSyncWrite.clearParam()
        #     time.sleep(0.7) #wait for dxl to get to their positions


        # print("Before flag", time.perf_counter())
        # time.sleep(1.0)
        # start_flag.value = 1  # Start both processes at the same time

        dxl_process.join()
        sensor_process.join()
        aggregator_process.join()
        dxl_control_process.join()
        icl_sensor_process.join()









    def run_trajectory(self):
        """
        run the gcode trajectory so far
        
        The idea here is that we want to sample the sensor at a set frequency (100Hz) but the time for
        dynamixels and stuff is not constant. 

        So we can sample the sensor as fast as possible then interpolate the dynamixel data to match the
        sensor data.
        """
        # TODO
        n_samples = 5000
        frequency = 100  # Hz
        loop_time = 1 / frequency
        loop_start_time = time.perf_counter()

        for i in range(n_samples):
            print(time.perf_counter())
            data = self.get_sensor_data()
            # print(data)
            # self.get_dxl_data()

            # data.append(self.present_pos[0])  # gantry x act in pulse counts
            # data.append(self.present_pos[1])  # gantry y1 act in pulse counts
            # data.append(self.present_pos[2])  # gantry y2 act in pulse counts
            # data.append(self.present_pos[3])  # gantry z act in pulse counts
            # data.append(self.present_pos[4])  # gantry theta act in pulse counts
            # data.append(self.present_pos[5])  # gantry phi act in pulse counts

            # logline = str(data[0])
            # for i in range(1, len(data)):
            #     logline = logline + ", " + str(data[i])

            # print(logline)

            # -- Loop timing -- #
            # Sleep is fine for this.  Busy wait is a little more accurate
            time.sleep(loop_start_time + (i + 1) * loop_time - time.perf_counter())

            # Sleep may have trouble waking thread on time

        loop_end_time = time.perf_counter()

        time_ellapsed = loop_end_time - loop_start_time
        samples_per_second = n_samples / time_ellapsed
        print("Samples per second: %d", samples_per_second)


    '''shutdown robot'''
    def shutdown(self):
        # close the dynamixel port
        self.portHandler.closePort()


def generate_trajectories(n_samples,y_pos_list,depth_start,depth_end,depth_safe):

    def generate_target(y_pos,depth,waiting_time):
        return [0,y_pos,depth,0,0,waiting_time,0]

    depth_range = depth_start-depth_end
    indices = list(range(n_samples))
    depths = [-(x*math.sqrt(depth_range)/(n_samples-1))**2+depth_start for x in indices]
    waiting_times = [math.ceil((-x+depth_start)*30+3) for x in depths]

    trajectories = []
    for y_pos in y_pos_list:
        for i in indices:
            trajectories.append(generate_target(y_pos,depth_safe,waiting_times[i]))
            trajectories.append(generate_target(y_pos,depths[i],waiting_times[i]))
        trajectories.append(generate_target(y_pos,depth_safe,waiting_times[i]))
    
    return trajectories

def generate_poking_oscillation(freq,n_sample_per_depth,depths,depth_safe):
    def generate_target(depth,waiting_time):
        return [0,-2.5,depth,0,0,waiting_time,0]
    waiting_time = 0.5/freq
    trajectories = []
    for depth in depths:
        for _ in range(n_sample_per_depth):
            trajectories.append(generate_target(depth_safe,waiting_time))
            trajectories.append(generate_target(depth,waiting_time))
    trajectories.append(generate_target(depth_safe,waiting_time))
    return trajectories

if __name__ == "__main__":
    config = Config(
        log_save_name = "poking_oscillation_10Hz",
        trajectory_filename="trajectories/ellipsoidTrajectory_2000_lowforceFA7.npy",
        UDP_IP="192.168.1.201",
        UDP_DEST="192.168.1.1",
        UDP_PORT=11223,
        baud_rate=9600,
    )
    
    robot = TrainingRobotController(config)

    # -3.9 and -3.8
    # -3.85
    # start trajectgory a little high

    # 4.85
    debug_traj = [[0.0, 0.0,  -3, 0.0, 0.0, 0.0, 1] for _ in range(100000)]
    debug_traj = [
        [ 0.0, 0.0, -3.00, 0.0, 0.0, 1.0, 0],
        [ 0.0, 0.0, -3.00, 0.0, 0.0, 1.0, 0],
        [ 0.0, 0.0, -4.0, 0.0, 0.0, 1.0, 0],
        [ 0.0, 0.0, -4.5, 0.0, 0.0, 1.0, 0],
        [ 0.0, 0.0, -4.7, 0.0, 0.0, 1.0, 0],
        [ 0.0, 0.0, -4.9, 0.0, 0.0, 1.0, 0],
        [ 0.0, 0.0, -5.1, 0.0, 0.0, 1.0, 0],
        [ 0.0, 0.0, -3.00, 0.0, 0.0, 1.0, 0],

    ]

    # robot.load_debug_trajectory(debug_traj)
    # robot.load_trajectory(n_commands = None)

    n_samples = 10
    y_pos_list = [0,-2.5]

    # depth_start = -3.84
    # depth_end = -5.1

    depth_start = -3.89
    depth_end = -3.93

    depth_safe = -3
    # trajectories = generate_trajectories(n_samples,y_pos_list,depth_start,depth_end,depth_safe)

    warmup = [[0.0, 0.0, -3.0, 0.0, 0.0, 0.1, 0.0] for _ in range(20)]
    trajectories = generate_poking_oscillation(10,10,[-4.0,-4.1,-4.2],-3.8)

    trajectories = warmup + trajectories
    # print(trajectories)
    robot.load_debug_trajectory(trajectories)

    
    if not robot.check_traj():
        print(len(robot.err_pts))
        # raise ValueError("Bad trajectory points found.")

    print("Starting trajectory.")
    # robot.run_trajectory()
    robot.run_trajectory_parallel()
    
    print("Done with trajectory, shutting down.")
    robot.shutdown()
