from yumipy import YuMiRobot
from yumi_utils import *
from yumi_cfg import *
import h5py
import time
import argparse


if __name__ == "__main__":
    parser = argparse.ArgumentParser()
    parser.add_argument("-c","--control_mode",type=int,default=3)
    parser.add_argument("-i","--input_h5_file",type=str,default='./h5_data/inference.h5')
    parser.add_argument("-g","--group_name",type=str,default='group1')
    parser.add_argument("-v","--speed",type=float,default=150)
    args = parser.parse_args()

    # Set control mode
    control_mode = args.control_mode
    # Create instance of yumi robot
    yumi_robot = YuMiRobot()
    # Test communication
    state = yumi_robot.left.get_state()
    # Read trajectory data from h5
    f = h5py.File(args.input_h5_file,'r')
    l_joint_angles = f[args.group_name+'/l_joint_angle_2'][:]
    r_joint_angles = f[args.group_name+'/r_joint_angle_2'][:]

    # Set speed
    yumi_robot.my_set_v(args.speed)
    # Go to the initial state
    l_joint_target = rad2degree(clamp_to_limits(l_joint_angles[0], yumi_leftarm_lower_limits, yumi_leftarm_upper_limits))
    r_joint_target = rad2degree(clamp_to_limits(r_joint_angles[0], yumi_rightarm_lower_limits, yumi_rightarm_upper_limits))
    l_joint_state = set_new_target(l_joint_target)
    r_joint_state = set_new_target(r_joint_target)
    ### 1. Unsync goto state
    yumi_robot.left.goto_state(l_joint_state)
    yumi_robot.right.goto_state(r_joint_state)
    ### 2. Sync goto state
    # yumi_robot.goto_state_sync(l_joint_state,r_joint_state)
    ### 3. Continuous goto state
    if control_mode==3:
        yumi_robot.left.joint_buffer_clear()
        yumi_robot.right.joint_buffer_clear()
    # Iterate over the trajectory
    L = len(l_joint_angles)
    for idx in range(1,L,1):
        l_joint_target = rad2degree(clamp_to_limits(l_joint_angles[idx], yumi_leftarm_lower_limits, yumi_leftarm_upper_limits))
        r_joint_target = rad2degree(clamp_to_limits(r_joint_angles[idx], yumi_rightarm_lower_limits, yumi_rightarm_upper_limits))
        l_joint_state = set_new_target(l_joint_target)
        r_joint_state = set_new_target(r_joint_target)
        if control_mode == 1:
            ### 1. Unsync goto state
            yumi_robot.left.goto_state(l_joint_state)
            yumi_robot.right.goto_state(r_joint_state)
        elif control_mode == 2:
            ### 2. Sync goto state
            print("Sync mode: go to state {}/".format(idx)+str(L))
            yumi_robot.goto_state_sync(l_joint_state, r_joint_state)
        elif control_mode == 3:
            ### 3. Continus goto state
            yumi_robot.left.joint_buffer_add_single(l_joint_state)
            yumi_robot.right.joint_buffer_add_single(r_joint_state)
            
        time.sleep(0.001)
    if control_mode == 3:
        print("Control mode is continuous goto state !!!!!!!!!!")
        ### 3. Continus goto state
        yumi_robot.left.joint_buffer_move(wait_for_res=False)
        yumi_robot.right.joint_buffer_move(wait_for_res=False)
        
        