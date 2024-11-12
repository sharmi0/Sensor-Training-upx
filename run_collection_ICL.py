# imports
import os
import numpy as np
import socket
import serial
import time
import csv
from datetime import datetime as dt
from dxl_comms import *
from dataclasses import dataclass
import tqdm
import multiprocess
from multiprocess import Value, Queue

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

# Robot class
class TrainingRobot:
    """
    Instantiates the Robot with port
    """
    dxl_ids = (1, 2, 3, 4, 6, 5)  # Actual ids are switched from the gantry labels

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

        new_pos = [0]*6
        for i in range(len(self.dxl_ids)):
            new_pos[i] = self.groupSyncRead.getData(self.dxl_ids[i], ADDR_PRESENT_POSITION, LEN_PRESENT_POSITION)

        # Clear syncwrite parameter storage
        self.groupSyncRead.clearParam()

        self.present_pos[0] = pulses_to_position_x(new_pos[0])
        self.present_pos[1] = pulses_to_position_y1(new_pos[1])
        self.present_pos[2] = pulses_to_position_y2(new_pos[2])
        self.present_pos[3] = pulses_to_position_z(new_pos[3])
        self.present_pos[4] = p2r(new_pos[4])
        self.present_pos[5] = p2r(new_pos[5])
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

# aggregation
def aggregator_worker(data_dir_name, sensor_queue, dxl_queue, sensor_frequency, dxl_frequency, done_flag, tare_flag, data_valid_flag):

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

    data_key = {"time": 0, "Fx": 1, "Fy": 2, "Fz": 3,
            "s1": 4, "s2": 5, "s3": 6, "s4": 7, "s5": 8, "s6": 9, "s7": 10, "s8": 11, "tof1": 12, "tof2": 13, "tof3": 14, "tof4": 15, 
            "x_act": 16, "y1_act": 17, "y2_act": 18, "z_act": 19, "theta_act": 20, "phi_act": 21, 
            "data_valid": 22}

    config = None

    def __init__(self, config):
        self.config = config
        self.robot = TrainingRobot(config)
        self.commands = [] # commands are tuples of gcode string and list of dynamixel positions
        self.err_pts = []
        self.setup_log()

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

        commands_roll = r2p(float(traj_data[3])) # theta <> roll
        commands_pitch = r2p(float(traj_data[4])) # phi <> pitch

        commands_tdwell = traj_data[5]
        commands_Itare = traj_data[6]

        # check to make sure all commands are within gantry lims
        if (
            (commands_y1 < y1_lims[0] or commands_y1 > y1_lims[1])
            or (commands_y2 < y2_lims[0] or commands_y2 > y2_lims[1])
            or (commands_z < z_lims[0] or commands_z > z_lims[1])
            or (
                commands_roll < ati_roll_lims[0] or commands_roll > ati_roll_lims[1]
            )
            or (commands_pitch < ati_pitch_lims[0] or commands_pitch > ati_pitch_lims[1])
        ):
            self.err_pts.append([commands_x, commands_y1, commands_y2, commands_z, commands_roll, commands_pitch])
        else:
            new_command = [commands_x, commands_y1, commands_y2, commands_z, commands_roll, commands_pitch, commands_tdwell, commands_Itare]
            print(new_command)
            # append commands
            self.commands.append(tuple(new_command))

    def load_trajectory(self, end_idx=None):
        trajectory = np.load(self.config.trajectory_filename)
        if end_idx is not None:
            trajectory = trajectory[:end_idx]
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
        
        start_time = time.perf_counter() + 1.0

        dxl_queue = Queue()
        sensor_queue = Queue()
        
        done_flag = Value('i', 0)
        tare_flag = Value('i', 0)
        data_valid_flag = Value('i', 1)
        
        dxl_process = multiprocess.Process(target=dxl_sample_worker, 
                args=(self.robot, start_time, dxl_loop_time, dxl_queue, done_flag))
        sensor_process = multiprocess.Process(target=sensor_sample_worker, 
                args=(self.robot, start_time, sensor_loop_time, sensor_queue, done_flag))
        aggregator_process = multiprocess.Process(target=aggregator_worker,
                args=(self.data_dir_name, sensor_queue, dxl_queue, self.config.sensor_frequency, self.config.dxl_frequency, done_flag, tare_flag, data_valid_flag))
        dxl_control_process = multiprocess.Process(target=dxl_control_worker,
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
        trajectory_filename="trajectories/mmibaTrajectory_1.npy",
        UDP_IP="192.168.1.201",
        UDP_DEST="192.168.1.1",
        UDP_PORT=11223,
        baud_rate=9600,
        sensor_frequency = 100,
        dxl_frequency = 5,
    )

    # initialize
    robot_controller = TrainingRobotController(config)

    # load and check traj
    # debug_traj = [[0.1, 0.1, 0.1, 0.1, 0.1, 0.1, 1] for _ in range(100)]
    # robot_controller.load_debug_trajectory(debug_traj)
    robot_controller.load_trajectory()
    if not robot_controller.check_traj():
        print(len(robot_controller.err_pts))
        # raise ValueError("Bad trajectory points found.")

    # run traj
    print("Starting trajectory.")
    robot_controller.run_trajectory_parallel()
    
    # shut down
    print("Done with trajectory, shutting down.")
    robot_controller.shutdown()
