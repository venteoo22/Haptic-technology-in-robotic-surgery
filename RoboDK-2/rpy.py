# Type help("robolink") or help("robodk") for more information
# Press F5 to run the script
# Documentation: https://robodk.com/doc/en/RoboDK-API.html
# Reference:     https://robodk.com/doc/en/PythonAPI/index.html
# Note: It is not required to keep a copy of this file, your python script is saved with the station
import serial
import time
import math
from enum import Enum

# RoboDK API: import the robolink library (bridge with RoboDK)
from robolink import *
# Robot toolbox: import the robodk library (robotics toolbox)
from robodk import *

# ------------------------------------------------------------------------------
# Connection
# ------------------------------------------------------------------------------

# Establish the connection on a specific port
arduino = serial.Serial('COM7', 115200, timeout=1)

# Lets bring some time to the system to stablish the connetction
time.sleep(2)

# Establish a link with the simulator
RDK = Robolink()

# ------------------------------------------------------------------------------
# Simulator setup
# ------------------------------------------------------------------------------

# Retrieve all items (object in the robodk tree)
# Define the "robot" variable with our robot (UR5e)
robot = RDK.Item('UR5e')

# Define the "tcp" variable with the TCP of Endowrist needle
tcp_tool = RDK.Item('TCP_Endowrist')

# Performs a quick check to validate items defined
if robot.Valid():
    print('Robot selected: ' + robot.Name())
if tcp_tool.Valid():
    print('Tool selected: ' + tcp_tool.Name())

# Robot Flange with respect to UR5e base Frame
print('Robot POSE is: ' + repr(robot.Pose()))
# Tool frame with respect to Robot Flange
print('Robot POSE is: ' + repr(robot.PoseTool()))
# Tool frame with respect to Tool frame
print('TCP pose is: ' + repr(tcp_tool.Pose()))

robot.setSpeed(10)

# ------------------------------------------------------------------------------
# Reference frame is fixed to TCP
#
# Data comunication
# ------------------------------------------------------------------------------
#


class Command(Enum):
    GET_RPW = b'\x01'


try:

    # Discard initial ESP32 message
    arduino.reset_input_buffer()

    while True:

        # Requesting data to Ardino
        arduino.write(Command.GET_RPW.value)

        # Storing received data
        roll_str = arduino.readline().strip()
        pitch_str = arduino.readline().strip()
        yaw_str = arduino.readline().strip()
        torque_str = arduino.readline().strip()
        torque1_str = arduino.readline().strip()
        torque2_str = arduino.readline().strip()
        s1_str_rob = arduino.readline().strip()
        s2_str_rob = arduino.readline().strip()
        s1_str_pinza = arduino.readline().strip()
        s2_str_pinza = arduino.readline().strip()
        # Convert variable values from string to float
        roll = float(roll_str)
        pitch = float(pitch_str)
        yaw = float(yaw_str)
        torque = float(torque_str)
        torque1 = float(torque1_str)
        torque2 = float(torque2_str)
        s1_rob = bool(int(s1_str_rob))
        s2_rob = bool(int(s2_str_rob))
        s1_pinza = bool(int(s1_str_pinza))
        s2_pinza = bool(int(s1_str_pinza))
        print(roll, pitch, yaw,torque, torque1, torque2, s1_rob, s2_rob, s1_pinza, s2_pinza)

        # Convert from degrees to radians R,P,Y angles
        R = math.radians(roll)
        P = math.radians(pitch)
        W = math.radians(yaw)
        X = 0
        Y = -60
        Z = 320

        # Calculate the POSE matrix (UR)
        # TODO: Complete the POSE matrix
        # You have to translate the tool to the (X,Y,Z) position and apply the
        # R,P,W rotations to the tool
        # The tool and IMU axis must be consistent
        pose_matrix = transl(X,Y,Z) * rotz(-W) * roty(-P) * rotx(R) * rotx(pi)

        tcp_tool_pose = tcp_tool.setPoseTool(pose_matrix)

        # TODO: Move linealy the robot with the S1 and S2 buttons through the Y axis
        if not s1_rob:
            approach = robot.Pose() * transl(0,100,0)
            robot.MoveL(approach, False)
            
        elif not s2_rob:
            approach = robot.Pose() * transl(0,-100,0)
            robot.MoveL(approach, False)




except KeyboardInterrupt:
    print("Communication stopped.")
    pass

# ------------------------------------------------------------------------------
# Disconnect Arduino
# ------------------------------------------------------------------------------
print("Disconnecting Arduino...")
arduino.close()

