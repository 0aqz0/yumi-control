import numpy as np
from yumipy import YuMiRobot
from yumipy import YuMiState
import argparse
from argparse import RawTextHelpFormatter
from yumi_utils import *

if __name__ == "__main__":
    parser = argparse.ArgumentParser(description=
        "Useful script for moving left arm to target pose,\n\
        Some Information about joint limits: \n\
        Joint1: -168.45 ~ 168.45 \n\
        Joint2: -143.24 ~ 42.97 \n\
        Joint7: -228.61 ~ 228.61 \n\
        Joint3: -168.45 ~ 168.45 \n\
        Joint4: -123.19 ~ 79.64 \n\
        Joint5: -289.92 ~ 289.92 \n\
        Joint6: -87.66  ~ 137.51",
        formatter_class=RawTextHelpFormatter
        )
    parser.add_argument('-n','--nargs', nargs='+',type=float)
    args = parser.parse_args()

    # Default settings
    joint_target = np.zeros([7])
    arg_list = []

    for _, value in args._get_kwargs():
        if value is not None:
            arg_list = value

    for i, val in enumerate(arg_list):
        joint_target[i] = val

    joint_state = set_new_target(joint_target)
    # Create instance of yumi robot
    yumi_robot = YuMiRobot()
    # Go to the initial state
    yumi_robot.right.goto_state(joint_state)