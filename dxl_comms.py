from src.dynamixel_sdk import *

# Control table address
ADDR_TORQUE_ENABLE      = 64               # Control table address is different in Dynamixel model
ADDR_PROFILE_VELOCITY   = 112
ADDR_GOAL_POSITION      = 116
ADDR_MOVING_STATUS      = 123
ADDR_PRESENT_POSITION   = 132
ADDR_DRIVE_MODE         = 10

# Data Byte Length
LEN_GOAL_POSITION       = 4
LEN_PRESENT_POSITION    = 4
LEN_MOVING_STATUS       = 1
LEN_PROFILE_VELOCITY    = 4
LEN_DRIVE_MODE          = 1

# Protocol version
PROTOCOL_VERSION            = 2.0               # See which protocol version is used in the Dynamixel

# Default setting
BAUDRATE                    = 2000000             # Dynamixel default baudrate : 57600
DEVICENAME                  = '/dev/ttyUSB0' #'COM11' #'/dev/ttyUSB0'    # Check which port is being used on your controller
                                                # ex) Windows: "COM1"   Linux: "/dev/ttyUSB0" Mac: "/dev/tty.usbserial-*"

TORQUE_ENABLE               = 1                 # Value for enabling the torque
TORQUE_DISABLE              = 0                 # Value for disabling the torque

#PROFILE_VELOCITY            = 1               # Maximum velocity of the profile (if drive mode is set to 0 which should be default)
PROFILE_VELOCITY            = 100               # Maximum velocity of the profile (if drive mode is set to 1)

DXL_MOVING_STATUS_THRESHOLD = 10                # Dynamixel moving status threshold
DRIVE_MODE                  = 0

# pulley diameter
PITCH_D = 28.01  # mm
MAX_COUNTS = 4095

#DXL Bounds
# x_lims = [0, 2048]
# y1_lims = [500, 3900]
# y2_lims = [145, 3600]
# z_lims = [800, 3340]
# ati_pitch_lims = [1000, 2800] # phi <> pitch
# ati_roll_lims = [1100, 2565] # theta <> roll

# for ICL sensor
Z_OFFSET = 1900 # dxl position when sensor touches pedestal

x_lims = [248, 3848]
y1_lims = [1398, 2698]
y2_lims = [1398, 2698]
z_lims = [1750, 2035] # TODO: change lower limit for actual testing
ati_pitch_lims = [2000, 2100]
ati_roll_lims = [2000, 2100]

# max_z_height = 2035
# zero_z_height = 1750
# new_x_range = [277, 3846] # should be symmetric about 2048
# new_y_range = [1540, 2687] # should also be symmetric about 2048

def initComms():
    import sys, tty, termios
    # import sys

    fd = sys.stdin.fileno()
    old_settings = termios.tcgetattr(fd)
    def getch():
        try:
            tty.setraw(sys.stdin.fileno())
            ch = sys.stdin.read(1)
        finally:
            termios.tcsetattr(fd, termios.TCSADRAIN, old_settings)
        return ch
    # Initialize PortHandler instance
    # Set the port path
    # Get methods and members of PortHandlerLinux or PortHandlerWindows
    portHandler = PortHandler(DEVICENAME)

    # Initialize PacketHandler instance
    # Set the protocol version
    # Get methods and members of Protocol1PacketHandler or Protocol2PacketHandler
    packetHandler = PacketHandler(PROTOCOL_VERSION)

    # Initialize GroupSyncWrite instance
    groupSyncWrite = GroupSyncWrite(portHandler, packetHandler, ADDR_GOAL_POSITION, LEN_GOAL_POSITION)

    # Initialize GroupSyncWrite instance
    groupSyncWrite_PROF_VEL = GroupSyncWrite(portHandler, packetHandler, ADDR_PROFILE_VELOCITY, LEN_PROFILE_VELOCITY)

    # Initialize GroupSyncRead instance for Present Position
    groupSyncRead = GroupSyncRead(portHandler, packetHandler, ADDR_PRESENT_POSITION, LEN_PRESENT_POSITION)

    # Open port
    if portHandler.openPort():
        print("Succeeded to open the port")
    else:
        print("Failed to open the port")
        print("Press any key to terminate...")
        getch()
        quit()

    # Set port baudrate
    if portHandler.setBaudRate(BAUDRATE):
        print("Succeeded to change the baudrate")
    else:
        print("Failed to change the baudrate")
        print("Press any key to terminate...")
        getch()
        quit()
        
    return portHandler, packetHandler, groupSyncWrite, groupSyncRead, groupSyncWrite_PROF_VEL

# # helpers 
# def r2p(rad):
#     return round(rad * 2048 / np.pi) + 2048  # radians to pulse counts

# def p2r(pulse):
#     return(pulse - 2048) * np.pi / 2048  # pulse counts to radians

# '''convert from position value to dxl pulse counts **X**'''    
# def position_to_pulses_x(position):
#     max_counts = 4095
#     return round(position * (max_counts/(np.pi*PITCH_D))) + 1997

# '''convert from position value to dxl pulse counts **Y1**'''    
# def position_to_pulses_y1(position):
#     max_counts = 4095
#     return 2098 - round(position * (max_counts/(np.pi*PITCH_D)))

# '''convert from position value to dxl pulse counts **Y2**'''    
# def position_to_pulses_y2(position):
#     max_counts = 4095
#     return round(position * (max_counts/(np.pi*PITCH_D))) + 1992

# '''convert from position value to dxl pulse counts **Z**'''    
# def position_to_pulses_z(position):
#     max_counts = 4095
#     z_offset = 1680 # was 1740
#     return round(position * (max_counts/(np.pi*PITCH_D))) + z_offset #set z offset to be such that 0 is where the sensor touches the pedestal


# '''convert from pulse counts to position values **X** '''    
# def pulses_to_position_x(counts):
#     max_counts = 4095
#     return (counts-1997) * (np.pi*PITCH_D)/max_counts

# '''convert from pulse counts to position values **Y1**'''    
# def pulses_to_position_y1(counts):
#     max_counts = 4095
#     return (2098-counts) * (np.pi*PITCH_D)/max_counts

# '''convert from pulse counts to position values **Y2**'''    
# def pulses_to_position_y2(counts):
#     max_counts = 4095
#     return (counts-1992) * (np.pi*PITCH_D)/max_counts

# '''convert from pulse counts to position values **Z**'''    
# def pulses_to_position_z(counts):
#     max_counts = 4095
#     return (counts-1740) * (np.pi*PITCH_D)/max_counts

'''convert from position value to dxl pulse counts **X**'''    
def position_to_pulses_x(position):
    return round(position * (MAX_COUNTS/(np.pi*PITCH_D))) + 2048

'''convert from position value to dxl pulse counts **Y1**'''    
def position_to_pulses_y1(position):
    return 2048 - round(position * (MAX_COUNTS/(np.pi*PITCH_D)))

'''convert from position value to dxl pulse counts **Y2**'''    
def position_to_pulses_y2(position):
    return round(position * (MAX_COUNTS/(np.pi*PITCH_D))) + 2048

'''convert from position value to dxl pulse counts **Z**'''    
def position_to_pulses_z(position):
    return round(position * (MAX_COUNTS/(np.pi*PITCH_D))) + Z_OFFSET #set z offset to be such that 0 is where the sensor touches the pedestal


'''convert from pulse counts to position values **X** '''    
def pulses_to_position_x(counts):
    return (counts-2048) * (np.pi*PITCH_D)/MAX_COUNTS

'''convert from pulse counts to position values **Y1**'''    
def pulses_to_position_y1(counts):
    return (2048-counts) * (np.pi*PITCH_D)/MAX_COUNTS

'''convert from pulse counts to position values **Y2**'''    
def pulses_to_position_y2(counts):
    return (counts-2048) * (np.pi*PITCH_D)/MAX_COUNTS

'''convert from pulse counts to position values **Z**'''    
def pulses_to_position_z(counts):
    return (counts-2048) * (np.pi*PITCH_D)/MAX_COUNTS