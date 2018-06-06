"""
Script to send the PWM commands from this computer to the Pi over ethernet
"""
#%% User Input Section

# IP address of Raspberry Pi
IP_Address = '192.168.4.1'
# '192.168.4.1' # Use when static

# Input maximum allowable thruster speed from 0-100% (MAX 35% for lab Tests)
max_thruster_speed = 30
# Note that each thruster draws ~1 Ampere at 35% max possible speed

#%% Import required modules and methods

from time import sleep # This allows a time delay between execution
import math # Allows more complex computations
import socket # Import socket module for data transmission
import pickle # Import serial converter for data transmission
import pygame # Import pygame library
import pygame.locals as PL
import webbrowser # Import camera control
import OpenGL.GL as GL
import OpenGL.GLU as GLU
import sys
sys.path.append('.')

#%% Functions

# Define round-off function to nearest 10 for stable PWM signal
def round10(num):
    if num%10 < 5:
        return num - num%10
    else:
        return num - num%10 + 10
    
# Define a function to find the magnitude of a vector (needed to normalise pos)
def mag(x):
    return math.sqrt(sum(i**2 for i in x))

# Define a function to convert cartesian to polar (application specific)
def polar(Rx,Ry):
    [Rr , Rth] = [0 , 0]
    if [Rx , Ry] == [0 , 0]:
        Rr = 0
    elif Rx < 0 and Ry == 0:
        [Rr , Rth] = [mag([Rx,Ry]) , math.pi/2]
    elif Rx > 0 and Ry == 0:
        [Rr , Rth] = [mag([Rx,Ry]) , 3*math.pi/2]
    elif Rx == 0 and Ry > 0:
        [Rr , Rth] = [mag([Rx,Ry]) , 0]
    elif Rx == 0 and Ry < 0:
        [Rr , Rth] = [mag([Rx,Ry]) , math.pi]
    elif Rx < 0 and Ry > 0:
        [Rr , Rth] = [mag([Rx,Ry]) , math.atan(-Rx/Ry)]
    elif Rx < 0 and Ry < 0:
        [Rr , Rth] = [mag([Rx,Ry]) , math.pi + math.atan(-Rx/Ry)]
    elif Rx > 0 and Ry < 0:
        [Rr , Rth] = [mag([Rx,Ry]) , math.pi + math.atan(-Rx/Ry)]
    elif Rx > 0 and Ry > 0:
        [Rr , Rth] = [mag([Rx,Ry]) , 2*math.pi + math.atan(-Rx/Ry)]
    else:
        Rr = 0
    return [Rr , Rth]

def resize(width, height):
    GL.glViewport(0, 0, width, height)
    GL.glMatrixMode(GL.GL_PROJECTION)
    GL.glLoadIdentity()
    GLU.gluPerspective(45.0, float(width) / height, 0.001, 10.0)
    GL.glMatrixMode(GL.GL_MODELVIEW)
    GL.glLoadIdentity()
    GLU.gluLookAt(0.0, 1.0, -5.0,
              0.0, 0.0, 0.0,
              0.0, 1.0, 0.0)              
    
def init():
    GL.glEnable(GL.GL_DEPTH_TEST)
    GL.glClearColor(0.0, 0.0, 0.0, 0.0)
    GL.glShadeModel(GL.GL_SMOOTH)
    GL.glEnable(GL.GL_BLEND)
    GL.glEnable(GL.GL_POLYGON_SMOOTH)
    GL.glHint(GL.GL_POLYGON_SMOOTH_HINT, GL.GL_NICEST)
    GL.glEnable(GL.GL_COLOR_MATERIAL)
    GL.glEnable(GL.GL_LIGHTING)
    GL.glEnable(GL.GL_LIGHT0)
    GL.glLightfv(GL.GL_LIGHT0, GL.GL_AMBIENT, (0.3, 0.3, 0.3, 1.0))
    
def scene1():
    GL.glClear(GL.GL_COLOR_BUFFER_BIT | GL.GL_DEPTH_BUFFER_BIT)
    GL.glColor((1.,1.,1.))
    GL.glLineWidth(1)
    GL.glBegin(GL.GL_LINES)

    for x in range(-20, 22, 2):
        GL.glVertex3f(x/10.,-1,-1)
        GL.glVertex3f(x/10.,-1,1)
        
    for x in range(-20, 22, 2):
        GL.glVertex3f(x/10.,-1, 1)
        GL.glVertex3f(x/10., 1, 1)
        
    for z in range(-10, 12, 2):
        GL.glVertex3f(-2, -1, z/10.)
        GL.glVertex3f( 2, -1, z/10.)

    for z in range(-10, 12, 2):
        GL.glVertex3f(-2, -1, z/10.)
        GL.glVertex3f(-2,  1, z/10.)

    for z in range(-10, 12, 2):
        GL.glVertex3f( 2, -1, z/10.)
        GL.glVertex3f( 2,  1, z/10.)

    for y in range(-10, 12, 2):
        GL.glVertex3f(-2, y/10., 1)
        GL.glVertex3f( 2, y/10., 1)
        
    for y in range(-10, 12, 2):
        GL.glVertex3f(-2, y/10., 1)
        GL.glVertex3f(-2, y/10., -1)
        
    for y in range(-10, 12, 2):
        GL.glVertex3f(2, y/10., 1)
        GL.glVertex3f(2, y/10., -1)
        
    GL.glEnd()


#%% Define objects

class Cube(object):

    def __init__(self, position):
        self.position = position

    # Cube information
    num_faces = 6

    vertices = [ (-1.0, 0.45, 0.5),
                 (1.0, 0.45, 0.5),
                 (1.0, 0.55, 0.5),
                 (-1.0, 0.55, 0.5),
                 (-1.0, 0.45, -0.5),
                 (1.0, 0.45, -0.5),
                 (1.0, 0.55, -0.5),
                 (-1.0, 0.55, -0.5) ]

    normals = [ (0.0, 0.0, +1.0),  # front
                (0.0, 0.0, -1.0),  # back
                (+1.0, 0.0, 0.0),  # right
                (-1.0, 0.0, 0.0),  # left
                (0.0, +1.0, 0.0),  # top
                (0.0, -1.0, 0.0) ]  # bottom

    vertex_indices = [ (0, 1, 2, 3),  # front
                       (4, 5, 6, 7),  # back
                       (1, 5, 6, 2),  # right
                       (0, 4, 7, 3),  # left
                       (3, 2, 6, 7),  # top
                       (0, 1, 5, 4) ]  # bottom

    def render(self):
        vertices = self.vertices

        # Draw all 6 faces of the cube
        GL.glBegin(GL.GL_QUADS)

        for face_no in xrange(self.num_faces):
            GL.glNormal3dv(self.normals[face_no])
            v1, v2, v3, v4 = self.vertex_indices[face_no]
            GL.glColor (1, 0, 0)
            GL.glVertex(vertices[v1])
            GL.glColor (0.1, 0.5, 1)
            GL.glVertex(vertices[v2])
            GL.glColor (0.1, 0.5, 1)            
            GL.glVertex(vertices[v3])
            GL.glColor (1, 0, 0)
            GL.glVertex(vertices[v4])
        GL.glEnd()


class Cube2(object):

    def __init__(self, position):
        self.position = position

    # Cube information
    num_faces = 6

    vertices = [ (-1.0, -0.55, 0.5),
                 (1.0, -0.55, 0.5),
                 (1.0, -0.45, 0.5),
                 (-1.0, -0.45, 0.5),
                 (-1.0, -0.55, -0.5),
                 (1.0, -0.55, -0.5),
                 (1.0, -0.45, -0.5),
                 (-1.0, -0.45, -0.5) ]

    normals = [ (0.0, 0.0, +1.0),  # front
                (0.0, 0.0, -1.0),  # back
                (+1.0, 0.0, 0.0),  # right
                (-1.0, 0.0, 0.0),  # left
                (0.0, +1.0, 0.0),  # top
                (0.0, -1.0, 0.0) ]  # bottom

    vertex_indices = [ (0, 1, 2, 3),  # front
                       (4, 5, 6, 7),  # back
                       (1, 5, 6, 2),  # right
                       (0, 4, 7, 3),  # left
                       (3, 2, 6, 7),  # top
                       (0, 1, 5, 4) ]  # bottom

    def render(self):
        vertices = self.vertices

        # Draw all 6 faces of the cube
        GL.glBegin(GL.GL_QUADS)

        for face_no in xrange(self.num_faces):
            GL.glNormal3dv(self.normals[face_no])
            v1, v2, v3, v4 = self.vertex_indices[face_no]
            GL.glColor (1, 0, 0)
            GL.glVertex(vertices[v1])
            GL.glColor (0.1, 0.5, 1)
            GL.glVertex(vertices[v2])
            GL.glColor (0.1, 0.5, 1)            
            GL.glVertex(vertices[v3])
            GL.glColor (1, 0, 0)
            GL.glVertex(vertices[v4])
        GL.glEnd()

#%% Connect to Raspberry Pi
    
port = 5560

try:
    sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    sock.connect((IP_Address, port))
    print('Connection successful.')
except:
    print('Connection failed.')
    sock.close()
    exit()

#%% Initialise Variables
    
pygame.display.init()
pygame.joystick.init()
j = pygame.joystick.Joystick(0)
j.init()

# Calculation of maximum allowable PWM signal range around 1500 deadband
max_PWM_range = (float(max_thruster_speed)/100)*400

# Initialisation of loop conditions
Init_Loop = False # Loop will repeat infinitely until 'True' or broken
Main_Loop = True # Loop will repeat infinitely until 'False' or broken

# Initialisation of IMU display settings
SCREEN_SIZE = (800, 600)
SCALAR = .5
SCALAR2 = 0.2

# Initialisation of PWM array with deadband frequencies
PWM_array = [1500 , 1500 , 1500 , 1500] # [FL , FR , BL , BR]

# Initialisation of floating PWM array with deadband frequencies
PWM_array_f = [1500 , 1500 , 1500 , 1500]


# Initialisation of cartesian position vector for xbox controller (right stick)
[Rx , Ry] = [0 , 0] # [rightX , rightY]

# Initialisation of polar position vector for xbox controller (right stick)
[Rr , Rth] = [0 , 0]
# Note that theta = 0 is defined as on positive y axis

# Initialisation of xbox dpad array
dpad_array = [0 , 0 , 0 , 0]

#%% Initialisation Loop   
while Init_Loop == False: # ESC waits for user to press 'Start'
    # Refresh controller settings
    pygame.event.pump()
    # Break pre-initialisation loop with 'Start' button
    if j.get_button(7) == 1:
        webbrowser.get().open('http://' + IP_Address + '/html') # Open IP of Pi
        sock.send(pickle.dumps('INITIALISE'))
        print 'Initialisation command sent.'
        pygame.init()
        pygame.display.set_mode(SCREEN_SIZE, PL.HWSURFACE | PL.OPENGL | PL.DOUBLEBUF)
        resize(*SCREEN_SIZE)
        init()
        cube = Cube((0.0, 0.0, 0.0))
        cube2 = Cube2((0.0, 0.0, 0.0))
        break
    else:
        pass
sleep(3) # Give ESCs time to initialise
    
#%% Main Propulsion Loop

while Main_Loop == True: # Thrusters will keep running until broken by 'Back'
    if j.get_button(6) == 1:
        sock.send(pickle.dumps('STOP'))
        print 'Stop command sent.'
        break
    else:
        # Refresh controller readings
        pygame.event.pump()
        # Update cartesian position vector
        [Rx , Ry] = [j.get_axis(4) , j.get_axis(3)]
        # Compute polar position vector
        [Rr , Rth] = polar(Rx,Ry)
        # Cap maximum vector magnitude at 1 to prevent overpowering
        if Rr > 1:
            Rr = 1
            
        # Convert dpad (modern pygame form) into conventional dpad variables
        if j.get_hat(0)[0] == 0 and j.get_hat(0)[1] == 0:
            dpadUp = 0
            dpadDown = 0
            dpadLeft = 0
            dpadRight = 0
        if j.get_hat(0)[0] == 1:
            dpadRight = 1
        if j.get_hat(0)[0] == -1:
            dpadLeft = 1
        if j.get_hat(0)[1] == 1:
            dpadUp = 1
        if j.get_hat(0)[1] == -1:
            dpadDown = 1
                
        # Update dpad array arrowkeys [Up , Down , Left , Right]
        dpad_array = [dpadUp , dpadDown , dpadLeft , dpadRight]
        
        if dpad_array == [0,0,0,0]:
            # Update array of floating PWM values
            PWM_array_f[0] = 1500 - max_PWM_range*j.get_axis(1)
            PWM_array_f[1] = 1500 + max_PWM_range*j.get_axis(1)
            if Rth >= 0 and Rth < math.pi/2:
                PWM_array_f[2] = 1500 - Rr*max_PWM_range
                PWM_array_f[3] = 1500 + Rr*max_PWM_range*math.cos(2*Rth)
            elif Rth >= math.pi/2 and Rth < math.pi:
                PWM_array_f[2] = 1500 + Rr*max_PWM_range*math.cos(2*Rth)
                PWM_array_f[3] = 1500 - Rr*max_PWM_range
            elif Rth >= math.pi and Rth < 3*math.pi/2:
                PWM_array_f[2] = 1500 + Rr*max_PWM_range
                PWM_array_f[3] = 1500 - Rr*max_PWM_range*math.cos(2*Rth)
            elif Rth >= 3*math.pi/2 and Rth < 2*math.pi:
                PWM_array_f[2] = 1500 - Rr*max_PWM_range*math.cos(2*Rth)
                PWM_array_f[3] = 1500 + Rr*max_PWM_range
                                                            
            # Update PWM array by rounding all values from previous to nearest 10
            for pulse in range(4):
                PWM_array[pulse] = int(round10(PWM_array_f[pulse]))
            
        else:
            # Use dpad to fine-tune rotational position
            PWM_array[0] = 1500 + 80*dpad_array[0] - 80*dpad_array[1]
            PWM_array[1] = 1500 - 80*dpad_array[0] + 80*dpad_array[1]
            PWM_array[2] = 1500 - 80*dpad_array[2] + 80*dpad_array[3]
            PWM_array[3] = 1500 - 80*dpad_array[2] + 80*dpad_array[3]
        
        # Send PWM and speed limit data to Raspberry Pi for execution
        serial_data = pickle.dumps(PWM_array)
        sock.send(serial_data)
        
        # Retrieve IMU data from Raspberry Pi
        values = pickle.loads(sock.recv(1024))
        
        # Update screen with IMU data
        for event in pygame.event.get():
            if event.type == PL.QUIT:
                PL.QUIT() # Dodgy step
            if event.type == PL.KEYUP and event.key == PL.K_ESCAPE:
                PL.KEYUP() and PL.K_ESCAPE() # Dodgy step

        x_angle = values[0]
        y_angle = values[1]
        z_angle = values[2]

        scene1()
        GL.glPushMatrix()
        GL.glRotate(-float(x_angle - 2.5), 0, 0, 1)
        GL.glRotate(float(y_angle + 7), 1, 0, 0)
        GL.glRotate(float(0), 0, 1, 0)
        cube.render()
        GL.glPopMatrix()
          
        
        GL.glPushMatrix()
        GL.glRotate(float(0), 0, 0, 1)
        GL.glRotate(float(0), 1, 0, 0)
        GL.glRotate(-float(z_angle), 0, 1, 0)
        cube2.render()
        GL.glPopMatrix()
        pygame.display.flip()
        
        # Receive bearing data
        heading = values[3]
        
        if heading < 22.5 and heading >= -22.5:
            bearing = 'West'
        elif heading >= 22.5 and heading < 67.5:
            bearing = 'South-West'
        elif heading >= 67.5 and heading < 112.5:
            bearing = 'South'
        elif heading >= 112.5 and heading < 157.5:
            bearing = 'South-East'
        elif heading >= 157.5 or heading < -157.5:
            bearing = 'East'
        elif heading >= -157.5 and heading < -112.5:
            bearing = 'North-East'
        elif heading >= -112.5 and heading < -67.5:
            bearing = 'North'
        elif heading >= -67.5 and heading < -22.5:
            bearing = 'North-West'
        
        print bearing , heading
        
        if heading == 'IMU Failed':
            print heading
            break
        
        # Give connection time to breathe
        sleep(0.1)

#%% Post-testing cleanup
        
# Close pygame
pygame.quit()
# Close the data transmission port
sock.close()
print 'Termination successful.'
