from yumipy import YuMiRobot
from yumi_control import *
from inspire_hand_control import InspireHand, HandControlThread
import h5py
import argparse


if __name__ == "__main__":
    parser = argparse.ArgumentParser()
    parser.add_argument("-c","--control_mode",type=int,default=3)
    parser.add_argument("-i","--input_h5_file",type=str,default='./h5_data/inference.h5')
    parser.add_argument("-g","--group_name",type=str,default='group1')
    parser.add_argument("-v","--speed",type=float,default=150)
    parser.add_argument("-d","--delta_time",type=float,default=0.2)
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
    l_glove_angles = f[args.group_name+'/l_glove_angle_2'][:]
    r_glove_angles = f[args.group_name+'/r_glove_angle_2'][:]

    # Init HAND controller & control thread
    left_hand_controller = InspireHand('COM12', 115200)
    right_hand_controller = InspireHand('COM13', 115200)
    left_hand_controller.setspeed(1000, 1000, 1000, 1000, 1000, 1000)
    right_hand_controller.setspeed(1000, 1000, 1000, 1000, 1000, 1000)
    left_hand_control_thread = HandControlThread(0, left_hand_controller, l_glove_angles, args.delta_time)
    right_hand_control_thread = HandControlThread(1, right_hand_controller, r_glove_angles, args.delta_time)

    # Set speed (e.g. v100) This is set for all traj path points since currentSpeed is fixed...
    yumi_robot.my_set_v(args.speed)
    # Set time interval (dt for execution of each path point)
    yumi_robot.my_set_dt(args.delta_time)
    # Go to the initial state
    l_joint_target = rad2degree(clamp_to_limits(l_joint_angles[0], yumi_leftarm_lower_limits, yumi_leftarm_upper_limits))
    r_joint_target = rad2degree(clamp_to_limits(r_joint_angles[0], yumi_rightarm_lower_limits, yumi_rightarm_upper_limits))
    l_joint_state = set_new_target(l_joint_target)
    r_joint_state = set_new_target(r_joint_target)
    ### 1. Unsync goto state
    yumi_robot.left.goto_state(l_joint_state, wait_for_res=False) # send move command for both
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
        l_joint_target = rad2degree(clamp_to_limits(l_joint_angles[idx],yumi_leftarm_lower_limits,yumi_leftarm_upper_limits))
        r_joint_target = rad2degree(clamp_to_limits(r_joint_angles[idx],yumi_rightarm_lower_limits,yumi_rightarm_upper_limits))
        l_joint_state = set_new_target(l_joint_target)
        r_joint_state = set_new_target(r_joint_target)
        if control_mode == 1:
            ### 1. Unsync goto state
            yumi_robot.left.goto_state(l_joint_state)
            yumi_robot.right.goto_state(r_joint_state)
        elif control_mode == 2:
            ### 2. Sync goto state
            print("Sync mode: go to state {}/".format(idx)+str(L))
            yumi_robot.goto_state_sync(l_joint_state,r_joint_state)
        elif control_mode == 3:
            ### 3. Continus goto state
            yumi_robot.left.joint_buffer_add_single(l_joint_state)
            yumi_robot.right.joint_buffer_add_single(r_joint_state)

    # 开启灵巧手控制线程
    left_hand_control_thread.start()
    right_hand_control_thread.start()
    # 添加线程到线程列表
    threads = []
    threads.append(left_hand_control_thread)
    threads.append(right_hand_control_thread)

    ### Start executing motions of arms and hands
    # Arms:
    if control_mode == 3:
        print("Control mode is continuous goto state !!!!!!!!!!")
        ### 3. Continus goto state
        yumi_robot.left.joint_buffer_move(wait_for_res=False)
        yumi_robot.right.joint_buffer_move(wait_for_res=False)
    # Hands:
    for t in threads:
        t.join()
