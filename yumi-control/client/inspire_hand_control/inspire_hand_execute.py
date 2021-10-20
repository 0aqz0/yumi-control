from inspire_hand import InspireHand
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


def hand_execute(hand_controller,glove_angle_traj,time_interval):
    exec_angle_traj = convert_optim_ang_exec_ang(glove_angle_traj)
    nData = len(exec_angle_traj)
    for i in range(nData):
        exec_angle = exec_angle_traj[i]
        hand_controller.setpos(exec_angle[0],exec_angle[1],exec_angle[2],exec_angle[3],exec_angle[4],exec_angle[5])
        time.sleep(time_interval)


def convert_optim_ang_exec_ang(optim_angles):
    # Input *optim_angles* is of the size (50, 12)
    # prep
    ndata = optim_angles.shape[0]
    elec_signal = np.zeros((ndata, 6), dtype="int32")
    optim_fourfin_range = [0, -1.6]
    optim_thumbroll_range = [0.1, 0.0]
    optim_thumbrot_range = [-1.0, 0.3]
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


if __name__=="__main__":
    parser = argparse.ArgumentParser()
    parser.add_argument("-i","--input_h5_file",type=str,default='./h5_data/inference.h5')
    parser.add_argument("-g","--group_name",type=str,default='group1')
    parser.add_argument("-d","--delta_time",type=float,default=0.2)
    args = parser.parse_args()

    f = h5py.File(args.input_h5_file,'r')
    l_glove_angles = f[args.group_name+'/l_glove_angle_2'][:]
    r_glove_angles = f[args.group_name+'/r_glove_angle_2'][:] 

    left_hand_controller = InspireHand('COM12', 115200)
    right_hand_controller = InspireHand('COM13', 115200)
    left_hand_controller.setspeed(1000, 1000, 1000, 1000, 1000, 1000)
    right_hand_controller.setspeed(1000, 1000, 1000, 1000, 1000, 1000)

    # 初始化灵巧手控制线程
    delta_time = 0.3
    left_hand_control_thread = HandControlThread(0, left_hand_controller, l_glove_angles, delta_time)
    right_hand_control_thread = HandControlThread(1, right_hand_controller, r_glove_angles, delta_time)

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
