"""Functions for modeling ROSBot"""

from math import cos, sin
import numpy as np

def model_parameters():
    """Returns two constant model parameters"""
    # This is a stub. Write your code here.
    # return param_k, param_d
    param_k = 1.0 #A scaling factor for the linear speed.
    param_d= 1.0 #A scaling factor for the angular speed.
    return param_k, param_d

def speed_to_twist(speed_left, Speed_right):
    """
    Given normalized speeds for the left and right wheels, return the linear and
    angular velocity of the robot.
    """
    param_k,param_d = model_parameters()# This function converts the left and right wheel speeds into the robot’s linear and angular velocities.
    speed_linear = (param_k/2)*(speed_left + Speed_right)
    speed_angular = (param_k/(2*param_d))*(speed_linear-speed_left)
    return speed_linear,speed_angular

def system_matrix(state_theta):
    """
    Returns a numpy array with the A(theta) matrix
    for a differential drive robot
    """
    # This is a stub. Write your code here.
    # return system_matrix_A
    A = np.array([[cos(state_theta),0],
                  [sin(state_theta),0
                   [0,1]]]) #This function computes the system matrix A(θ) for the robot’s kinematic mode
    return A

def system_field(state_z, input_u): #This function computes the state derivatives (i.e., the rate of change of the robot’s state).
    """Computes the field at a given state for the dynamical model"""
    # This is a stub. Write your code here.
    # return dot_state_z
    theta = state_z[2]
    A = system_matrix(theta)
    dot_state_Z= A @ input_u
    return dot_state_Z

def euler_step(state_z, input_u, step_size): #This function performs one step of numerical integration using Euler’s method to update the robot’s state.
    """Integrates the dynamical model for one time step using Euler's method"""
    # This is a stub. Write your code here.
    # return state_z_next
    dot_state_Z = system_field(state_z,input_u)
    state_z_next = state_z+dot_state_Z*step_size
    return state_z_next

def twist_to_speeds(speed_linear, speed_angular):#This function converts the desired linear and angular velocities into left and right wheel speeds.
    """
    Given the desired linear and angular velocity of the robot, returns
    normalized speeds for the left and right motors. Speeds needs to be
    thresholded to be between −1.0 (backward at maximum speed) and 1.0
    (forward at maximum speed).
    """
    # This is a stub. Write your code here.
    # return speed_left, speed_right
    speed_left = speed_linear - speed_angular
    speed_right = speed_linear + speed_angular
    speed_left = max(-1.0, min(1.0, speed_left))
    speed_right = max(-1.0, min(1.0, speed_right))
    return speed_left, speed_right

class KeysToVelocities(): # This class translates keyboard inputs into speed commands for the robot. update_speeds(): Updates the robot’s linear and angular speeds based on key presses.
    '''
    Class to translate cumulative key strokes to speed commands
    '''
    def __init__(self):
        # initialize attributes here
        # This is a stub. Write your code here.
        self.speed_linear = 0
        self.speed_angular = 0
        self.text_description = 'Initial values'

    def update_speeds(self, key):
        # This is a stub. Insert your code here
        # This is a stub. Write your code here.
        if key == ord('W'): # Move Forward
            self.speed_linear +=0.1
        elif key == ord('S'): #Move Backwards
            self.speed_linear -=0.1
        elif key ==  ord('A'): # Move Left
            self.speed_angular +=0.1
        elif key == ord('D'): # Move Right
             self.speed_angular -=0.1
        elif key(''): # Stop
             self.speed_linear = 0
        self.speed_angular = 0

        #Thershold speed to be within [-1.0,1.0]
        self.speed_linear =max(-1.0, min(1.0,self.speed_linear))
        self.speed_angular = max(-1.0, min(1.0,self.speed_angular))

        self.text_description = f'Linear: {self.speed_linear:.2f}, Angular: {self.speed_angular:.2f}'
        return self.speed_linear, self.speed_angular, self.text_description

class StampedMsgRegister(): #This class stores a previous message and computes the delay between the current and previous messages.
    '''
    Store a previous message, and compute delay with respect to current one
    '''
    def __init__(self):
        # initialize attributes here
        self.msg_previous = None
        self.time_previous = None
       # pass

    def replace_and_compute_delay(self, msg):
        '''
        Compute delay between current and stored message,
        then replace stored message with current one
        '''
        # return time_delay, msg_previous
        time_current = msg.header.stamp.to_sec()
        if self.time_previous is None:
            delay =0.0
        else:
            delay = time_current -self.time_previous
        self.msg_previous =msg
        self.time_previous =time_current
        return delay,self.msg_previous

    def previous_stamp(self):
        '''
        Return stored message
        '''
        return self.msg_previous
        # return stamp
