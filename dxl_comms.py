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

#DXL Bounds
x_lims = [0, 2048]
y1_lims = [500, 3900]
y2_lims = [145, 3600]
z_lims = [800, 3340]
ati_pitch_lims = [1000, 2800]
ati_roll_lims = [1100, 2565]

def initComms():
    import sys, tty, termios
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