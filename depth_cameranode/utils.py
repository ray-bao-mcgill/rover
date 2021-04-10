import math
import numpy as np
from pyquaternion import Quaternion

def euler_to_quaternion(roll, pitch, yaw):
# Source https://computergraphics.stackexchange.com/questions/8195/how-to-convert-euler-angles-to-quaternions-and-get-the-same-euler-angles-back-fr
        qx = np.sin(roll/2) * np.cos(pitch/2) * np.cos(yaw/2) - np.cos(roll/2) * np.sin(pitch/2) * np.sin(yaw/2)
        qy = np.cos(roll/2) * np.sin(pitch/2) * np.cos(yaw/2) + np.sin(roll/2) * np.cos(pitch/2) * np.sin(yaw/2)
        qz = np.cos(roll/2) * np.cos(pitch/2) * np.sin(yaw/2) - np.sin(roll/2) * np.sin(pitch/2) * np.cos(yaw/2)
        qw = np.cos(roll/2) * np.cos(pitch/2) * np.cos(yaw/2) + np.sin(roll/2) * np.sin(pitch/2) * np.sin(yaw/2)

        return np.array([qx, qy, qz, qw])

def quaternion_to_euler(x, y, z, w):

    t0 = +2.0 * (w * x + y * z)
    t1 = +1.0 - 2.0 * (x * x + y * y)
    roll = math.atan2(t0, t1)
    t2 = +2.0 * (w * y - z * x)
    t2 = +1.0 if t2 > +1.0 else t2
    t2 = -1.0 if t2 < -1.0 else t2
    pitch = math.asin(t2)
    t3 = +2.0 * (w * z + x * y)
    t4 = +1.0 - 2.0 * (y * y + z * z)
    yaw = math.atan2(t3, t4)
    return [yaw, pitch, roll]

DEFAULT_ROTATION = euler_to_quaternion(math.radians(-90), 0, 0)
class Pose:
    def __init__(self, position, quaternion):
        # Set initial position of rover as center of coordinate grid
        self.init_pos = np.array([position.x, position.y, position.z])
        self.current_pos = np.array([position.x, position.y, position.z])
        self.init_quaternion = Quaternion(quaternion.w, quaternion.x, quaternion.y, quaternion.z)
        self.current_quaternion = Quaternion(quaternion.w, quaternion.x, quaternion.y, quaternion.z)

    def update_current_pos(self, position):
        self.current_pos[0] = position.x
        self.current_pos[1] = position.y
        self.current_pos[2] = position.z
    
    def update_current_quaternion(self, quaternion):
        self.current_quaternion = Quaternion(quaternion.w, quaternion.x, quaternion.y, quaternion.z)
        
    def get_current_offset(self):
        '''Return current offset from initial position of rover'''
        return self.current_pos - self.init_pos

    def get_current_inverse_rotation(self):
        return self.current_quaternion * self.init_quaternion.inverse

class Point:
    def __init__(self, x, y, z):
        self.x = x 
        self.y = y 
        self.z = z