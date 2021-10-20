#encoding=utf-8
import sys
sys.path.append("..")
from inspire_hand import InspireHand
from math import sin, pi
import time
import numpy as np
import threading
import h5py
import argparse

class HandControlThread(threading.Thread):
    def __init__(self,threadID,hand_controller,glove_angle_traj,time_interval):
        threading.Thread.__init__(self)
        self.threadID = threadID
        self.hand_controller = hand_controller
        self.glove_angle_traj = glove_angle_traj
        self.time_interval = time_interval

    def run(self):
        hand_execute(self.hand_controller, self.glove_angle_traj, self.time_interval)#,self.threadID)

def hand_execute(hand_controller,glove_angle_traj,time_interval): #,id):
    ## Sine movement for testing purpose
    # t = np.linspace(0,1000,5000)
    # L = len(t)
    # for i in range(L):
    #     x = int(500*sin(2*pi*t[i]/500)+500)
    #     hand_controller.setpos(x,x,x,x,x,x)
    #     time.sleep(0.002)

    exec_angle_traj = convert_optim_ang_exec_ang(glove_angle_traj)
    nData = len(exec_angle_traj)
    for i in range(nData):
        exec_angle = exec_angle_traj[i] #[2000,2000,2000,2000,2000,2000] #
        # if id == 0:
        #     print("left pinky:{}".format(exec_angle[0]))
        # else:
        #     print("right pinky:{}".format(exec_angle[0]))
        hand_controller.setpos(exec_angle[0],exec_angle[1],exec_angle[2],exec_angle[3],exec_angle[4],exec_angle[5])
        time.sleep(time_interval)


def convert_optim_ang_exec_ang(optim_angles):
    # Input *optim_angles* is of the size (50, 12)
    # prep
    ndata = optim_angles.shape[0]
    elec_signal = np.zeros((ndata, 6), dtype="int32")
    optim_fourfin_range = [0, -1.6]
    optim_thumbroll_range = [0.1, 0.0]
    optim_thumbrot_range = [-1.0, 0.3] #[0.3, -1.0] # actually fixed at intermediate position when we still use S14 glove, since it doesn't measure this axis of motion
    max_elec = 2000
    min_elec = 0

    # conversion
    for i in range(ndata):
        # four fingers (for inspire hand electrical signal, order is: pinky->ring->middle->index)
        elec_signal[i, 3] = round((optim_angles[i, 0] - optim_fourfin_range[0]) / (optim_fourfin_range[1] - optim_fourfin_range[0]) * (max_elec - min_elec))
        elec_signal[i, 2] = round((optim_angles[i, 2] - optim_fourfin_range[0]) / (optim_fourfin_range[1] - optim_fourfin_range[0]) * (max_elec - min_elec))
        elec_signal[i, 1] = round((optim_angles[i, 4] - optim_fourfin_range[0]) / (optim_fourfin_range[1] - optim_fourfin_range[0]) * (max_elec - min_elec))
        elec_signal[i, 0] = round((optim_angles[i, 6] - optim_fourfin_range[0]) / (optim_fourfin_range[1] - optim_fourfin_range[0]) * (max_elec - min_elec))
        # thumb roll
        elec_signal[i, 4] = round((optim_angles[i, 9] - optim_thumbroll_range[0]) / (optim_thumbroll_range[1] - optim_thumbroll_range[0]) * (max_elec - min_elec))
        # thumb rot (actually fixed when using S14 generated data)
        elec_signal[i, 5] = round((optim_angles[i, 8] - optim_thumbrot_range[0]) / (optim_thumbrot_range[1] - optim_thumbrot_range[0]) * (max_elec - min_elec))

        # check if within range
        for j in range(6):
            if elec_signal[i, j] < 0 or elec_signal[i, j] > 2000:
                print("Error: Joint angle J{}={} of path point {} is out of bounds!".format(i, optim_angles[i, j], j))
                return None

    return elec_signal

def hack_glove_angles(glove_angles):
    glove_angles[:,9] = 0.02
    return glove_angles

if __name__=="__main__":
    parser = argparse.ArgumentParser()
    parser.add_argument("-c","--control_mode",type=int,default=3)
    parser.add_argument("-i","--input_h5_file",type=str,default='../h5_data/optim_trajs_for_demo_real_robot/mocap_ik_results_YuMi_g2o_similarity.h5')
    parser.add_argument("-g","--group_name",type=str,default='fengren_1')#'kai_3')
    parser.add_argument("-v","--speed",type=float,default=100)
    parser.add_argument("-hl","--hand_length",type=int,default=500)

    args = parser.parse_args()

    # hand_len = 500
    f = h5py.File(args.input_h5_file,'r')
    l_glove_angles = f[args.group_name+'/l_glove_angle_2'][:]
    r_glove_angles = f[args.group_name+'/r_glove_angle_2'][:] 
    # l_glove_angles = hack_glove_angles(l_glove_angles)
    # r_glove_angles = hack_glove_angles(r_glove_angles)

    left_hand_controller = InspireHand('/dev/ttyUSB1',115200)
    right_hand_controller = InspireHand('/dev/ttyUSB0',115200)
    # left_hand_controller.setspeed(500,500,500,500,500,500)
    # right_hand_controller.setspeed(500,500,500,500,500,500)
    left_hand_controller.setspeed(1000,1000,1000,1000,1000,1000)
    right_hand_controller.setspeed(1000,1000,1000,1000,1000,1000)

    # 初始化灵巧手控制线程
    delta_time = 0.03
    left_hand_control_thread = HandControlThread(0,left_hand_controller,l_glove_angles,delta_time)
    right_hand_control_thread = HandControlThread(1,right_hand_controller,r_glove_angles,delta_time)

    # 开启灵巧手控制线程
    left_hand_control_thread.start()
    right_hand_control_thread.start()

    # 添加线程到线程列表
    threads = []
    threads.append(left_hand_control_thread)
    threads.append(right_hand_control_thread)
    
    # 等待所有线程完成
    for t in threads:
        t.join()

    print("Done!")

    # left_hand_controller.setpos(1000,1000,1000,1000,1000,1000)
    # time.sleep(1)
    # left_hand_controller.setpos(0,0,0,0,0,0)