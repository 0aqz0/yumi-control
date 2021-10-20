import numpy as np
from yumipy import YuMiState

def rad2degree(angle):
    return angle * 180.0 / np.pi

def clamp_to_limits(joint_angle, lower_bound, upper_bound):
    return np.clip(joint_angle, lower_bound, upper_bound)

def set_new_target(target):
    state = YuMiState()
    state.joint1 = target[0]
    state.joint2 = target[1]
    state.joint7 = target[2]
    state.joint3 = target[3]
    state.joint4 = target[4]
    state.joint5 = target[5]
    state.joint6 = target[6]
    return state