# imports
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

import multiprocessing
from multiprocessing import Value, Queue

# Config
@dataclass
class Config:
    log_save_name: str
    trajectory_filename: str

    UDP_IP: str  # IP of the computer running the script
    UDP_DEST: str  # IP of Nucleo Board
    UDP_PORT: int 

    baud_rate: str

    sensor_frequency: int
    dxl_frequency: int
    num_sensor_samples: int
    traj_time: int # todo: make float?




# helpers 
def r2p(rad):
    return round(rad * 2048 / np.pi) + 2048  # radians to pulse counts

def p2r(pulse):
    return(pulse - 2048) * np.pi / 2048  # pulse counts to radians

PITCH_D = 28.01  # mm

'''convert from position value to dxl pulse counts **X**'''    
def position_to_pulses_x(position):
    max_counts = 4095
    return round(position * (max_counts/(np.pi*PITCH_D))) + 1997

'''convert from position value to dxl pulse counts **Y1**'''    
def position_to_pulses_y1(position):
    max_counts = 4095
    return 2098 - round(position * (max_counts/(np.pi*PITCH_D)))

'''convert from position value to dxl pulse counts **Y2**'''    
def position_to_pulses_y2(position):
    max_counts = 4095
    return round(position * (max_counts/(np.pi*PITCH_D))) + 1992

'''convert from position value to dxl pulse counts **Z**'''    
def position_to_pulses_z(position):
    max_counts = 4095
    z_offset = 1680 # was 1740
    return round(position * (max_counts/(np.pi*PITCH_D))) + z_offset #set z offset to be such that 0 is where the sensor touches the pedestal


'''convert from pulse counts to position values **X** '''    
def pulses_to_position_x(counts):
    max_counts = 4095
    return (counts-1997) * (np.pi*PITCH_D)/max_counts

'''convert from pulse counts to position values **Y1**'''    
def pulses_to_position_y1(counts):
    max_counts = 4095
    return (2098-counts) * (np.pi*PITCH_D)/max_counts

'''convert from pulse counts to position values **Y2**'''    
def pulses_to_position_y2(counts):
    max_counts = 4095
    return (counts-1992) * (np.pi*PITCH_D)/max_counts

'''convert from pulse counts to position values **Z**'''    
def pulses_to_position_z(counts):
    max_counts = 4095
    return (counts-1740) * (np.pi*PITCH_D)/max_counts


# Robot class
class TrainingRobot:
    """
    Instantiates the Robot with port
    """
    dxl_ids = (1, 2, 3, 4, 6, 5)  # Actual ids are switched from the gantry labels

    x_lims = dxl_comms.x_lims
    y1_lims = dxl_comms.y1_lims
    y2_lims = dxl_comms.y2_lims
    z_lims = dxl_comms.z_lims
    ati_theta_lims = dxl_comms.ati_roll_lims
    ati_phi_lims = dxl_comms.ati_pitch_lims

    config = None

    def __init__(self, config):
        self.config = config
        self.setup_ethernet()
        self.setup_dxl()

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
        self.present_pos = [0, 0, 0, 0, 0, 0]

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

        self.present_pos[0] = pulses_to_position_x(self.present_pos[0])
        self.present_pos[1] = pulses_to_position_y1(self.present_pos[1])
        self.present_pos[2] = pulses_to_position_y2(self.present_pos[2])
        self.present_pos[3] = pulses_to_position_z(self.present_pos[3])
        self.present_pos[4] = p2r(self.present_pos[4])
        self.present_pos[5] = p2r(self.present_pos[5])
        return self.present_pos
    
    def get_icl_data(self):
        """Pull data from icl sensor over serial"""

    '''shutdown robot'''
    def shutdown(self):
        # close the dynamixel port
        self.portHandler.closePort()

# Functions for multiprocesssing

# sampling
def sensor_sample_worker(robot, start_time, loop_time, data_queue, done_flag):
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
        sample = robot.get_sensor_data()
        # sample = [time.perf_counter()]
        data_queue.put(sample)
        # all_data.append(sample)
        i += 1
    print(overrun_count)

def dxl_sample_worker(robot, start_time, loop_time, data_queue, done_flag):
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
        sample = robot.get_dxl_data()
        # sample = [time.perf_counter()]
        data_queue.put(sample)
        # all_data.append(sample)
        i += 1
    print(overrun_count)

# aggregation tests
def aggregator_ati_worker(data_dir_name, sensor_queue, sensor_frequency, done_flag, data_valid_flag):
    sensor_data = None
    all_data = []
    data_valid = 0 
    i = 0
    while (done_flag.value == 0):
        try:
            sensor_data = sensor_queue.get(block=True, timeout=10)
        except:
            print("Lost data collection")
            break
        with data_valid_flag.get_lock():
            data_valid = data_valid_flag.value
        combo_data = sensor_data + [data_valid] # dxl data is just the most recent one
        for j in range(1, 12):
            print(f"{sensor_data[j]:+.4f}", end = " ")
        print(len(all_data))
        all_data.append(combo_data)
        if (len(all_data) % 1000 == 0):
            data_filename = os.path.join(data_dir_name, f"segment_{i}.npy")
            print(data_filename)
            np.save(data_filename, all_data)
            all_data = []
            print("Saved")
        i += 1
    data_filename = os.path.join(data_dir_name, f"segment_{i}.npy")
    np.save(data_filename, all_data)
    if done_flag.value == 0:
        raise Exception("QUEUE TIMEDOUT")


def aggregator_dxl_worker(data_dir_name, dxl_queue, sensor_frequency, dxl_frequency, done_flag, data_valid_flag):
    sensor_period = 1.0 / sensor_frequency
    dxl_period = int(sensor_frequency / dxl_frequency)
    sensor_data = None
    dxl_data = None
    all_data = []
    data_valid = 0 
    i = 0
    while (done_flag.value == 0):
        try:
            if i % dxl_period == 0:
                dxl_data = dxl_queue.get(block=True, timeout=10)
        except:
            print("Lost data collection")
            break
        combo_data = dxl_data # dxl data is just the most recent one
        all_data.append(combo_data)
        if (len(all_data) % 1000 == 0):
            data_filename = os.path.join(data_dir_name, f"segment_{i}.npy")
            print(data_filename)
            np.save(data_filename, all_data)
            all_data = []
            print("Saved")
        i += 1
        time.sleep(sensor_period)
    data_filename = os.path.join(data_dir_name, f"segment_{i}.npy")
    np.save(data_filename, all_data)
    if done_flag.value == 0:
        raise Exception("QUEUE TIMEDOUT")



# aggregation
def aggregator_worker(data_dir_name, sensor_queue, dxl_queue, n_sensor_samples, sensor_frequency, dxl_frequency, done_flag, tare_flag, data_valid_flag):

    dxl_period = int(sensor_frequency / dxl_frequency)
    sensor_data = None
    dxl_data = None

    all_data = []
    data_valid = 0 
    i = 0
    while (done_flag.value == 0):
        try:
            sensor_data = sensor_queue.get(block=True, timeout=10)

            if i % dxl_period == 0:
                dxl_data = dxl_queue.get(block=True, timeout=10)
        except:
            print("Lost data collection")
            break

        # print("hi")
        with data_valid_flag.get_lock():
            data_valid = data_valid_flag.value
        
        combo_data = sensor_data + dxl_data + [data_valid] # dxl data is just the most recent one

        # print(sensor_data[4:12])
        for j in range(1, 12):
            print(f"{sensor_data[j]:+.4f}", end = " ")
        print(len(all_data))
        #         tare_flag.value = 0
        #         all_data.append([-1 for _ in range(len(combo_data))])
        all_data.append(combo_data)

        if (len(all_data) % 1000 == 0):
            data_filename = os.path.join(data_dir_name, f"segment_{i}.npy")
            print(data_filename)
            np.save(data_filename, all_data)
            all_data = []
            # np.save(self.data_filename, all_data)
            # np.save(self.key_filename, self.data_key)
            print("Saved")
        i += 1
    

    data_filename = os.path.join(data_dir_name, f"segment_{i}.npy")
    np.save(data_filename, all_data)

    if done_flag.value == 0:
        raise Exception("QUEUE TIMEDOUT")


# motor control
def dxl_control_worker(robot, commands, done_flag, tare_flag, data_valid_flag):
    
    pbar = tqdm.tqdm(enumerate(commands), total=len(commands))
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

        if i == 0 or dxlt_des != commands[i-1][4] or dxlp_des != commands[i-1][5]:
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
        for i in range(len(robot.dxl_ids)):
            # print("DXL ID: ", self.dxl_ids[i], "DXL_COMMAND: ", dxl_commands[i])
            param_goal_position = [DXL_LOBYTE(DXL_LOWORD(dxl_commands[i])), DXL_HIBYTE(DXL_LOWORD(dxl_commands[i])), DXL_LOBYTE(DXL_HIWORD(dxl_commands[i])), DXL_HIBYTE(DXL_HIWORD(dxl_commands[i]))]
            dxl_addparam_result = robot.groupSyncWrite.addParam(robot.dxl_ids[i], param_goal_position)

        # Syncwrite goal position
        dxl_comm_result = robot.groupSyncWrite.txPacket()
        time.sleep(robot.dxl_delay)

        # Clear syncwrite parameter storage
        robot.groupSyncWrite.clearParam()
        time.sleep(t_dwell) #wait for dxl to get to their positions
    
    done_flag.value = 1


# Create Controller
class TrainingRobotController:
    """
    Instantiates the Robot Controller around Robot
    """

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
        self.robot = TrainingRobot(config)

        self.setup_commands()
        self.setup_log()

    def setup_commands(self):
        # setup initial commands to robot
        # commands are tuples of gcode string and list of dynamixel positions
        self.commands = []

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
        

    def add_point(self,traj_data): 
        """add a trajectory point"""
        # x, y, z, theta, phi, t_dwell, I_tare

        # calculate dynamixel positions
        # traj data = x, y, z, theta, phi
        commands_x = position_to_pulses_x(float(traj_data[0]))
        commands_y1 = position_to_pulses_y1(float(traj_data[1]))
        commands_y2 = position_to_pulses_y2(float(traj_data[1]))
        commands_z = position_to_pulses_z(float(traj_data[2]))

        commands_theta = r2p(float(traj_data[3]))
        commands_phi = r2p(float(traj_data[4]))

        commands_tdwell = traj_data[5]
        commands_Itare = traj_data[6]

        # check to make sure all commands are within gantry lims
        if (
            (commands_y1 < self.robot.y1_lims[0] or commands_y1 > self.robot.y1_lims[1])
            or (commands_y2 < self.robot.y2_lims[0] or commands_y2 > self.robot.y2_lims[1])
            or (commands_z < self.robot.z_lims[0] or commands_z > self.robot.z_lims[1])
            or (
                commands_theta < self.robot.ati_theta_lims[0] or commands_theta > self.robot.ati_theta_lims[1]
            )
            or (commands_phi < self.robot.ati_phi_lims[0] or commands_phi > self.robot.ati_phi_lims[1])
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

    
    def load_trajectory(self, n_commands):
        trajectory = np.load(self.config.trajectory_filename)
        if n_commands is not None:
            trajectory = trajectory[:n_commands]
        for command in trajectory:
            self.add_point(command)


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
    
    def run_trajectory_original(self):
        
        # define loop times
        sensor_loop_time = 1 / self.config.sensor_frequency
        dxl_loop_time = 1 / self.config.dxl_frequency

        # number of sensor loops
        n_sensor_samples = self.config.traj_time / self.config.sensor_frequency
        # number of dxl loops
        n_dxl_samples = self.config.traj_time / self.config.dxl_frequency
        # number of sensor samples per dxl loop
        n_sensor_samples_per_dxl = int(n_sensor_samples / n_dxl_samples)

        # start timer, keep iterating until all sensor samples are collected
        for i in range(n_dxl_samples):
            # send dxl commands
            new_command = self.commands[i]
            dxlx_des = (new_command[0])
            dxly1_des = (new_command[1])
            dxly2_des = (new_command[2])
            dxlz_des = (new_command[3]) 
            dxlt_des = (new_command[4])
            dxlp_des = (new_command[5])
            # t_dwell = new_command[6]
            # I_tare = new_command[7]
            dxl_commands = [dxlx_des, dxly1_des, dxly2_des, dxlz_des, dxlt_des, dxlp_des]      
            #command new goal position all
            for i in range(len(self.robot.dxl_ids)):
                # print("DXL ID: ", self.dxl_ids[i], "DXL_COMMAND: ", dxl_commands[i])
                param_goal_position = [DXL_LOBYTE(DXL_LOWORD(dxl_commands[i])), DXL_HIBYTE(DXL_LOWORD(dxl_commands[i])), DXL_LOBYTE(DXL_HIWORD(dxl_commands[i])), DXL_HIBYTE(DXL_HIWORD(dxl_commands[i]))]
                dxl_addparam_result = self.robot.groupSyncWrite.addParam(self.robot.dxl_ids[i], param_goal_position)
            # Syncwrite goal position
            dxl_comm_result = self.robot.groupSyncWrite.txPacket()
            time.sleep(self.robot.dxl_delay)
            # Clear syncwrite parameter storage
            self.robot.groupSyncWrite.clearParam()
            # time.sleep(t_dwell) #wait for dxl to get to their positions

            # now, get sensor data
            for _ in range(n_sensor_samples_per_dxl):
                # get sensor data
                sensor_data = self.robot.get_sensor_data()
                # get dxl data
                dxl_data = self.robot.get_dxl_data()
                # for now, just print the data
                print("\033c", end="")
                print("%5.2f, % 3.2f, % 3.2f, % 3.2f" % (sensor_data[0], sensor_data[1], sensor_data[2], sensor_data[3]))
                print(dxl_data)
                # wait for next loop
                time.sleep(sensor_loop_time)


    def run_traj_par_read_ati_only(self):

        sensor_loop_time = 1 / self.config.sensor_frequency        
        start_time = time.perf_counter() + 1.0

        sensor_queue = Queue()
        
        done_flag = Value('i', 0)
        tare_flag = Value('i', 0)
        data_valid_flag = Value('i', 1)
        
        sensor_process = multiprocessing.Process(target=sensor_sample_worker, 
                args=(self.robot, start_time, sensor_loop_time, sensor_queue, done_flag))
        aggregator_process = multiprocessing.Process(target=aggregator_ati_worker,
                args=(self.data_dir_name, sensor_queue, self.config.sensor_frequency, done_flag, data_valid_flag))

        # -- Start data collection -- #
        print('Starting sensor data process.')
        sensor_process.start()
        print('Starting aggregator process.')
        aggregator_process.start()
        time.sleep(1.0)
        # join all the processes
        print('Finishing processes.')
        sensor_process.join()
        aggregator_process.join()


    def run_traj_par_dxl_ctrl_only(self):
        dxl_loop_time = 1 / self.config.dxl_frequency        
        start_time = time.perf_counter() + 1.0

        dxl_queue = Queue()
        
        done_flag = Value('i', 0)
        tare_flag = Value('i', 0)
        data_valid_flag = Value('i', 1)
        
        dxl_process = multiprocessing.Process(target=dxl_sample_worker, 
                args=(self.robot, start_time, dxl_loop_time, dxl_queue, done_flag))
        aggregator_process = multiprocessing.Process(target=aggregator_dxl_worker,
                args=(self.data_dir_name, dxl_queue, self.config.sensor_frequency, self.config.dxl_frequency, done_flag, data_valid_flag))
        dxl_control_process = multiprocessing.Process(target=dxl_control_worker,
                                                      args=(self.robot, self.commands, done_flag, tare_flag, data_valid_flag))
        # -- Start data collection -- #
        print('Starting dxl data process.')
        dxl_process.start()
        print('Starting aggregator process.')
        aggregator_process.start()
        time.sleep(1.0)
        # start motor control
        print('Starting dxl control process.')
        dxl_control_process.start()
        # join all the processes
        print('Finishing processes.')
        dxl_process.join()
        aggregator_process.join()
        dxl_control_process.join()


    def run_trajectory_parallel(self):
        """
        run the gcode trajectory so far
        
        The idea here is that we want to sample the sensor at a set frequency (100Hz) but the time for
        dynamixels and stuff is not constant. 

        So we can sample the sensor as fast as possible then interpolate the dynamixel data to match the
        sensor data.
        """

        sensor_loop_time = 1 / self.config.sensor_frequency
        dxl_loop_time = 1 / self.config.dxl_frequency

        n_sensor_samples = int(self.config.traj_time * self.config.sensor_frequency)
        n_dxl_samples = int((self.config.dxl_frequency / self.config.sensor_frequency) * n_sensor_samples)
        
        start_time = time.perf_counter() + 1.0

        dxl_queue = Queue()
        sensor_queue = Queue()
        
        done_flag = Value('i', 0)
        tare_flag = Value('i', 0)
        data_valid_flag = Value('i', 1)
        
        dxl_process = multiprocessing.Process(target=dxl_sample_worker, 
                args=(self.robot, start_time, dxl_loop_time, dxl_queue, done_flag))
        sensor_process = multiprocessing.Process(target=sensor_sample_worker, 
                args=(self.robot, start_time, sensor_loop_time, sensor_queue, done_flag))
        aggregator_process = multiprocessing.Process(target=aggregator_worker,
                args=(self.data_dir_name, sensor_queue, dxl_queue, n_sensor_samples, self.config.sensor_frequency, self.config.dxl_frequency, done_flag, tare_flag, data_valid_flag))

        dxl_control_process = multiprocessing.Process(target=dxl_control_worker,
                                                      args=(self.robot, self.commands, done_flag, tare_flag, data_valid_flag))


        # -- Start data collection -- #
        print('Starting dxl data process.')
        dxl_process.start()
        print('Starting sensor data process.')
        sensor_process.start()
        print('Starting aggregator process.')
        aggregator_process.start()
        time.sleep(1.0)
        # start motor control
        print('Starting dxl control process.')
        dxl_control_process.start()
        # join all the processes
        print('Finishing processes.')
        dxl_process.join()
        sensor_process.join()
        aggregator_process.join()
        dxl_control_process.join()

    '''shutdown robot'''
    def shutdown(self):
        # shutdow robot
        self.robot.shutdown()




# run
if __name__ == "__main__":
    config = Config(
        log_save_name = "DEBUG",
        trajectory_filename="trajectories/ellipsoidTrajectory_2000_lowforceFA7.npy",
        UDP_IP="192.168.1.201",
        UDP_DEST="192.168.1.1",
        UDP_PORT=11223,
        baud_rate=9600,

        sensor_frequency = 100,
        dxl_frequency = 10,
        num_sensor_samples = 20000, # TODO: change to seconds of data
        traj_time = 20, # in seconds

    )

    robot_controller = TrainingRobotController(config)

    # # test dxl reading first
    # for idx in range(100):
    #     print(idx)
    #     dxl_test_data = robot_controller.robot.get_dxl_data()
    #     time.sleep(0.1)
    #     ati_test_data = robot_controller.robot.get_sensor_data()
    #     time.sleep(0.1)
    #     print(dxl_test_data, ati_test_data)
    #     time.sleep(1.0)

    # time.sleep(5.0)

    debug_traj = [[0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1] for _ in range(100)]

    robot_controller.load_debug_trajectory(debug_traj)
    # robot_controller.load_trajectory(n_commands = None)
    
    if not robot_controller.check_traj():
        print(len(robot_controller.err_pts))
        # raise ValueError("Bad trajectory points found.")

    print("Starting trajectory.")
    # robot_controller.run_traj_par_read_ati_only()
    robot_controller.run_traj_par_dxl_ctrl_only()
    # robot_controller.run_trajectory_parallel()
    
    print("Done with trajectory, shutting down.")
    robot_controller.shutdown()
