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
    verbose_aggregator: bool
    sensor_sample_freq: int
    dxl_sample_freq: int
    dxl_z_offset: int # dxl position when sensor touches pedestal, in counts
    dxl_roll_offset: int
    dxl_pitch_offset: int
    home_x_offset: float
    home_y_offset: float

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
        self.setup_mmiba()
        self.setup_dxl()

    def setup_ethernet(self):
        print("Creating ethernet socket for ati sampling.")
        self.sock = socket.socket(
            socket.AF_INET,  # Internet
            socket.SOCK_DGRAM,
        )  # UDP
        self.sock.bind((self.config.UDP_IP, self.config.UDP_PORT))

    def setup_mmiba(self):
        self.mmiba_ser = serial.Serial(
            port='/dev/tty.usbmodem21303',
            baudrate=921600,
            timeout=1
        )
        if self.mmiba_ser.isOpen():
            print(f"Serial port {self.mmiba_ser.port} is open.")
        else:
            print(f"Failed to open serial port {self.mmiba_ser.port}.")
            exit()

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

    def get_ati_data(self):
        tosend = "request"
        self.sock.sendto(tosend.encode(), (self.config.UDP_DEST, self.config.UDP_PORT))
        # receive data from system
        data, addr = self.sock.recvfrom(1024) # buffer size is 1024 bytes
        # decode data
        str_data = str(data.decode()).split('\n')[0]  # Recover first part of buffer
        float_data = [float(d) for d in str_data.split(",")]
        # only return time, fx, fy, fz
        return float_data[0:4]
    
    def get_mmiba_data(self):
        """Pull data from mmiba sensor over serial"""

        # TODO: do this with try except, better way to handle case when data dimension changes unexpectedly?
        float_data = [0.0]*36
        bad_data = 0
        # self.mmiba_ser.flush()
        data = self.mmiba_ser.readline()
        try:
            split_data = data.decode().strip().split(",")
            if (len(split_data) != 37):
                print("Bad serial data from mmiba: wrong length.")
                bad_data = 1
                # self.mmiba_ser.flush()
            else:
                for d in range(len(split_data)-1):
                    float_data[d] = float(split_data[d])
        except:
            print("Bad serial data from mmiba: cannot decode.")
            bad_data = 1
            # self.mmiba_ser.flush()

        return float_data, bad_data

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

    '''shutdown robot'''
    def shutdown(self):
        # close the dynamixel port
        self.portHandler.closePort()

# Functions for multiprocesssing

# sampling
def ati_sample_worker(robot, start_time, loop_time, ati_queue, done_flag):
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
        sample = robot.get_ati_data()
        # mmiba_sample = robot.get_mmiba_data()
        # sample = ati_sample + mmiba_sample # ati data is 4 values, mmiba data is 36 values
        ati_queue.put(sample)
        i += 1
    print("ATI sample loop overruns: ", overrun_count)

def mmiba_sample_worker(robot, start_time, loop_time, mmiba_queue, done_flag):
    overrun_count = 0
    bad_data_count = 0
    i = 0

    good_sample = [0.0]*36
    while done_flag.value == 0:

        sample, bad_data = robot.get_mmiba_data()

        sample_time = start_time + i * loop_time - time.perf_counter()

        if not bad_data:
            good_sample = sample
        else:
            bad_data_count += 1

        if sample_time < 0:
            mmiba_queue.put(good_sample)
            i += 1
        else:
            continue

        # if sample_time < 0 and -sample_time < loop_time:
        #     print(f"Loop overran for {sample_time}")
        #     overrun_count += 1
        #     if overrun_count / i > 0.01:  # want less than 1 percent to be overrun
        #         raise ValueError("Overran too much")
        # elif sample_time > 0:
        #     mmiba_queue.put(sample)
        # else:
        #     continue



        # Wait for the next sample time
        # sleep_time = start_time + i * loop_time - time.perf_counter()


        # if sleep_time > 0:
        #     time.sleep(sleep_time)
        # elif sleep_time < 0 and -sleep_time < loop_time:
        #     print(f"Loop overran for {sleep_time}")
        #     overrun_count += 1
        #     if overrun_count / i > 0.01:  # want less than 1 percent to be overrun
        #         raise ValueError("Overran too much")
        # # Always try to sample data
        # sample, bad_data = robot.get_mmiba_data()
        # bad_data_count += bad_data


        # # Based on sample time, put the data in the queue
        # sample_time = start_time + i * loop_time - time.perf_counter()
        # # if sample_time < 0 and -sample_time < loop_time:      
        # mmiba_queue.put(sample)
        # i += 1
    print("mmiba sample loop overruns: ", overrun_count)
    print("mmiba bad data count: ", bad_data_count)

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
        data_queue.put(sample)
        i += 1
    print("DXL sample loop overruns: ", overrun_count)

# aggregation
def aggregator_worker(data_dir_name, ati_queue, mmiba_queue, dxl_queue, dxl_command_queue, sensor_sample_freq, dxl_sample_freq, done_flag, tare_flag, data_valid_flag, verbose_flag):

    dxl_period = int(sensor_sample_freq / dxl_sample_freq)
    ati_data = None
    mmiba_data = None
    dxl_data = None

    dxl_command_data = [0, 0, 0, 0]

    all_data = []
    data_valid = 0 
    i = 0
    j = 0
    
    while (done_flag.value == 0):
        try:
            ati_data = ati_queue.get(block=True, timeout=10)
        except:
            print("Lost ATI data collection!")
            # continue

        try:
            mmiba_data = mmiba_queue.get(block=True, timeout=10)
        except:
            print("Lost mmiba data collection!")
            # continue

        try:
            if i % dxl_period == 0:
                dxl_data = dxl_queue.get(block=True, timeout=10)
        except:
            print("Lost DXL data collection!")
            # continue

        try:
            dxl_command_data = dxl_command_queue.get(block=False, timeout=10)
        except:
            # do nothing
            pass

        with data_valid_flag.get_lock():
            data_valid = data_valid_flag.value
        
        combo_data = ati_data + mmiba_data + dxl_data + [data_valid] + dxl_command_data # dxl data is just the most recent one

        # sensor data is now 4 ati values plus 36 mmiba values
        if verbose_flag:
            for j in range(1, 40): # essentially don't print time
                print(f"{combo_data[j]:+.4f}", end = " ")
            print(len(all_data))
        #         tare_flag.value = 0
        #         all_data.append([-1 for _ in range(len(combo_data))])
        all_data.append(combo_data)

        if (len(all_data) % 1000 == 0):
            data_filename = os.path.join(data_dir_name, f"segment_{j}.npy")
            print("Saving: ", data_filename)
            np.save(data_filename, all_data)
            all_data = []
            j += 1
        i += 1
    
    data_filename = os.path.join(data_dir_name, f"segment_{j}.npy")
    print("Saving: ", data_filename)
    np.save(data_filename, all_data)

    if done_flag.value == 0:
        raise Exception("QUEUE TIMEDOUT")


# motor control
def dxl_control_worker(robot, commands, dxl_command_queue, done_flag, tare_flag, data_valid_flag):
    
    # pbar = tqdm.tqdm(enumerate(commands), total=len(commands))
    data_valid = True
    for i, command in enumerate(commands):
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
        point_idx = command[8]
        
        dxl_commands = [dxlx_des, dxly1_des, dxly2_des, dxlz_des, dxlt_des, dxlp_des]
        # print("DXL COMMAND: ", dxl_commands)
        # print(dxl_commands)

        # put some of the relevant commands in the queue
        command_info = [pulses_to_position_x(dxlx_des), pulses_to_position_y1(dxly1_des), pulses_to_position_z(dxlz_des), point_idx]
        dxl_command_queue.put(command_info)

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
        time.sleep(t_dwell) # wait for dxl to get to their positions
    

    print("Done with trajectory.")
    done_flag.value = 1


# Create Controller
class TrainingRobotController:
    """
    Instantiates the Robot Controller around Robot
    """

    data_key = {"time": 0, "Fx": 1, "Fy": 2, "Fz": 3,
            "s1_1": 4,  "s1_2": 5,  "s1_3": 6,  "s1_4": 7,  "s1_5": 8,  "s1_6": 9,  "s1_7": 10, "s1_8": 11, "s1_9": 12, "s1_10": 13, "s1_11": 14, "s1_12": 15,
            "s2_1": 16, "s2_2": 17, "s2_3": 18, "s2_4": 19, "s2_5": 20, "s2_6": 21, "s2_7": 22, "s2_8": 23, "s2_9": 24, "s2_10": 25, "s2_11": 26, "s2_12": 27,
            "s3_1": 28, "s3_2": 29, "s3_3": 30, "s3_4": 31, "s3_5": 32, "s3_6": 33, "s3_7": 34, "s3_8": 35, "s3_9": 36, "s3_10": 37, "s3_11": 38, "s3_12": 39,
            "x_act": 40, "y1_act": 41, "y2_act": 42, "z_act": 43, "theta_act": 44, "phi_act": 45, 
            "data_valid": 46,
            "x_des": 47, "y_des": 48, "z_des": 49, "point_idx": 50}

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
        commands_x = position_to_pulses_x(float(traj_data[0]) + self.config.home_x_offset)
        commands_y1 = position_to_pulses_y1(float(traj_data[1]) + self.config.home_y_offset)
        commands_y2 = position_to_pulses_y2(float(traj_data[1]) + self.config.home_y_offset)
        commands_z = position_to_pulses_z(float(traj_data[2]), self.config.dxl_z_offset)

        commands_roll = r2p(float(traj_data[3])) + self.config.dxl_roll_offset # theta <> roll
        commands_pitch = r2p(float(traj_data[4])) + self.config.dxl_pitch_offset # phi <> pitch

        commands_tdwell = traj_data[5]
        commands_Itare = traj_data[6]

        commands_point_idx = traj_data[7]

        # check to make sure all commands are within gantry lims
        x_command_err = (commands_x < x_lims[0] or commands_x > x_lims[1])
        y_command_err = (commands_y1 < y1_lims[0] or commands_y1 > y1_lims[1]) or (commands_y2 < y2_lims[0] or commands_y2 > y2_lims[1])
        z_command_err = (commands_z < z_lims[0])
        if (commands_z > z_lims[1]):
            commands_z = z_lims[1]
        roll_command_err = (commands_roll < ati_roll_lims[0] or commands_roll > ati_roll_lims[1])
        pitch_command_err = (commands_pitch < ati_pitch_lims[0] or commands_pitch > ati_pitch_lims[1])
        if (x_command_err or y_command_err or z_command_err or roll_command_err or pitch_command_err):
            self.err_pts.append([commands_x, commands_y1, commands_y2, commands_z, commands_roll, commands_pitch])
            if x_command_err:
                print("X command out of bounds.")
            if y_command_err:
                print("Y command out of bounds.")
            if z_command_err:
                print("Z command out of bounds.")
            if roll_command_err:
                print("Roll command out of bounds.")
            if pitch_command_err:
                print("Pitch command out of bounds.")
        else:
            new_command = [commands_x, commands_y1, commands_y2, commands_z, commands_roll, commands_pitch, commands_tdwell, commands_Itare, commands_point_idx]
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
            print("No bad points, can proceed with trajectory.")
            return True
        else:
            print("Bad points found in trajectory, stopping.")
            print(self.err_pts)
            return False
    
    def run_trajectory_parallel(self):
        """
        run the gcode trajectory so far
        
        The idea here is that we want to sample the sensor at a set frequency (100Hz) but the time for
        dynamixels and stuff is not constant. 

        So we can sample the sensor as fast as possible then interpolate the dynamixel data to match the
        sensor data.
        """

        sensor_loop_time = 1 / self.config.sensor_sample_freq
        dxl_loop_time = 1 / self.config.dxl_sample_freq
        
        start_time = time.perf_counter() + 1.0

        dxl_queue = Queue()
        ati_queue = Queue()
        mmiba_queue = Queue()
        dxl_command_queue = Queue()

        done_flag = Value('i', 0)
        tare_flag = Value('i', 0)
        data_valid_flag = Value('i', 1)
        
        dxl_process = multiprocess.Process(target=dxl_sample_worker, 
                args=(self.robot, start_time, dxl_loop_time, dxl_queue, done_flag))
        ati_process = multiprocess.Process(target=ati_sample_worker, 
                args=(self.robot, start_time, sensor_loop_time, ati_queue, done_flag))
        mmiba_process = multiprocess.Process(target=mmiba_sample_worker, 
                args=(self.robot, start_time, sensor_loop_time, mmiba_queue, done_flag))
        aggregator_process = multiprocess.Process(target=aggregator_worker,
                args=(self.data_dir_name, ati_queue, mmiba_queue, dxl_queue, dxl_command_queue, self.config.sensor_sample_freq, self.config.dxl_sample_freq,
                        done_flag, tare_flag, data_valid_flag, self.config.verbose_aggregator))
        dxl_control_process = multiprocess.Process(target=dxl_control_worker,
                                                      args=(self.robot, self.commands, dxl_command_queue, done_flag, tare_flag, data_valid_flag))
        # -- Start data collection -- #
        print('Starting dxl data process.')
        dxl_process.start()
        print('Starting sensor data process.')
        ati_process.start()
        mmiba_process.start()
        print('Starting aggregator process.')
        aggregator_process.start()
        time.sleep(1.0)
        # start motor control
        print('Starting dxl control process.')
        dxl_control_process.start()
        # join all the processes
        dxl_process.join()
        ati_process.join()
        mmiba_process.join()
        aggregator_process.join()
        dxl_control_process.join()
        print('Finished processes.')

    '''shutdown robot'''
    def shutdown(self):
        # shutdow robot
        self.robot.shutdown()


# run
if __name__ == "__main__":
    config = Config(
        log_save_name = "SINGLEPRESS_BP_COMBO",
        trajectory_filename="trajectories/mmibaTrajectorysingle_press_2.npy",
        UDP_IP="192.168.1.201",
        UDP_DEST="192.168.1.1",
        UDP_PORT=11223,
        baud_rate=9600,
        verbose_aggregator = False,
        sensor_sample_freq = 100,
        dxl_sample_freq = 5,
        dxl_z_offset = 1800, # dxl position when sensor touches pedestal, in counts (1635 for vytaflex, 1750 for ecoflex)
        dxl_roll_offset = 15, # in counts
        dxl_pitch_offset = 0, # in counts
        home_x_offset = -0.5, # in mm
        home_y_offset = -1.0, # in mm
    )

    # initialize
    robot_controller = TrainingRobotController(config)

    # load traj
    # debug_traj = [[0.0, 0.0, 0.0, 0.0, 0.0, 0.1, 1, 20] for _ in range(10)]
    # robot_controller.load_debug_trajectory(debug_traj)
    robot_controller.load_trajectory()

    # check traj, then run
    if robot_controller.check_traj():
        # run traj
        print("Starting trajectory.")
        robot_controller.run_trajectory_parallel()
        
    # shut down
    print("Done with trajectory, shutting down.")
    robot_controller.shutdown()
