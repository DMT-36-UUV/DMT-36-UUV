"""
Script to receive PWM commands and excute them on GPIOs to power thrusters
"""

#%% Import required modules and methods

from RPIO import PWM
from time import sleep # This allows a time delay between execution
import socket # Import socket module for data transmission
import pickle # Import serial converter for data transmission
import pygame
import sys
import RTIMU
import os.path
import math
sys.path.append('.')

#%% Functions

def round10(num):
    if num%10 < 5:
        return num - num%10
    else:
        return num - num%10 + 10
    
def setupServer():
    sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    print('Socket created.')
    try:
        sock.bind((host, port))
    except socket.error as msg:
        print(msg)
    print('Socket bind complete.')
    return sock

def setupConnection():
    server.listen(-1) # Allows infinite connections at a time
    conn, address = server.accept() # Accept whatever the listening picks up on
    print('Connected to: ' + address[0] + ':' + str(address[1]))
    return conn

   
# Importanrt IMU reading loop
def read_values():
    SETTINGS_FILE = "RTIMULib"

    if not os.path.exists(SETTINGS_FILE + ".ini"):
        print("Settings file does not exist, will be created")

    settings = RTIMU.Settings(SETTINGS_FILE)
    imu = RTIMU.RTIMU(settings)

    if (not imu.IMUInit()):
        print("IMU Init Failed")
        conn.sendall(pickle.dumps('IMU Failed'))
        sys.exit(1)
        
    # Setup fusion parameters
    imu.setSlerpPower(0.02)
    imu.setGyroEnable(True)
    imu.setAccelEnable(True)
    imu.setCompassEnable(True)

    poll_interval = imu.IMUGetPollInterval()
    mag_declination = -0.4
      
    while True:      
        if imu.IMURead():
            data = imu.getIMUData()
            fusionPose = data["fusionPose"]
            compass = data ['compass']
            
            R_angle = math.degrees(fusionPose[0]) 
            P_angle = math.degrees(fusionPose[1])
            Y_angle = math.degrees(fusionPose[2])
            MagX = compass [0]
            MagY = compass [1]
            heading = math.degrees(math.atan2(MagY, MagX))
            heading = heading + mag_declination

            sleep(poll_interval*0.01/1000.0)
            return R_angle, P_angle, Y_angle, heading

#%% Initialise Variables

host = '' # Initialise host
port = 5560 # Arbitrary choice. Must be a free port.
s = PWM.Servo() # Redeclare servo variable

# GPIO Pins 17, 18, 22, 27 for PWM Signal and Pin 14 for Ground
# Assign short thruster IDs FL (Front-Left), FR, BL (Back-Left) and BR
FL = 17 # Assign Front-Left thruster to pin 17 (R6C1)
FR = 18 # Assign Front-Right thruster to pin 18 (R6C2)
BL = 22 # Assign Back-Left thruster to pin 22 (R8C1)
BR = 27 # Assign Back-Right thruster to pin 27 (R7C1)
all_thrusters = [FL , FR , BL , BR] # Create array of all thruster pins
# Note that the ground pin 14 is located at R7C2

# Initialisation of PWM array with zero frequency
PWM_array = [0 , 0 , 0 , 0] # [FL , FR , BL , BR]

# Initialisation of max_PWM_range variable
max_PWM_range = 120

# PID Control settings
pid_p = 0
pid_i = 0
pid_d = 0
previous_error = 0
elapsedTime = 0.1 # Should be 0 but use 0.1 to prevent dividing by 0 error

kp = 3.55
ki = 0.005
kd = 2.05

desired_angle = -5

#%% Run Function

def run(conn):
    pid_i = 0
    previous_error = 0
    elapsedTime = 0.1
    while True:
        # Receive the data
        serial_data = conn.recv(1024) # Receive the data, arbitrary buffer size
        serial_data = serial_data.decode('utf-8')
        # Translate the data from Serial into PWM array
        data = pickle.loads(serial_data)
        # Break loop if 'STOP' is sent from client
        if data == 'INITIALISE':
            print('Initialising thrusters.')
            for pin in all_thrusters:
                s.set_servo(pin,1500) # Initialise ESCs with deadband frequency
            sleep(3) # Give connection time to breathe before receiving PWM range
        elif data == 'STOP':
            break
        elif len(data) == 4:
            # Read values from IMU and send to computer
            values = read_values()
            conn.sendall(pickle.dumps(values))
            
            # Obtain raw user command data and adjust for auto-balancing
            PWM_array = data
            
            Angle_I_care=read_values()[1] 
            error = Angle_I_care - desired_angle
            pid_p = kp*error
            if -30 < error and error < 30:
                pid_i = pid_i+(ki*error)
    
            pid_d = kd*((error - previous_error)/elapsedTime)
            PID = pid_p + pid_i + pid_d
    
            if PID < -60:
                PID = -60
            elif PID > 60:
                PID = 60
            
            # Modify thrusters with PID adjustment values
            PWM_array[0] = PWM_array[0] - round10(PID)
            PWM_array[1] = PWM_array[1] - round10(PID)
        
            # Cap PWM array at specfied values
            for value in PWM_array:
                if value == 1500:
                    pass
                elif value > 1500:
                    if value > 1500 + max_PWM_range:
                        value = 1500 + max_PWM_range
                elif value < 1500:
                    if value < 1500 - max_PWM_range:
                        value = 1500 - max_PWM_range
                else:
                    print 'Unexpected PWM value of ' + value
                    break
            
            # Send PWM signals out to the GPIO pins
            for pin in range(4):
                s.set_servo(all_thrusters[pin] , PWM_array[pin])
                
            # Update error in roll angle
            previous_error = error
            
            # Update elapsed time
            sleep(0.1)
            elapsedTime = elapsedTime + 0.1
            
        else:
            print('Data transmission error.')
            break
    # Restore ESCs with deadband frequency
    for pin in all_thrusters:
        s.set_servo(pin,1500)
    pygame.quit()
    conn.close()
    server.close()
    print('Shutdown successful.')

#%% Internal run command

# Setup the server
server = setupServer()

# Connect to PC and run the main loop
while True:
    try:
        conn = setupConnection()
        run(conn)
    except:
        break
exit()
