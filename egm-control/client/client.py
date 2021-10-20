import socket
import time
import egm_pb2
import h5py
import numpy as np
from inspire_hand import InspireHand

# IP address & ports
CLIENT_IP = "192.168.125.10"
CLIENT_PORT_L = 6510
CLIENT_PORT_R = 6511

# inspire hand port name
left_hand_port = 'COM12'
right_hand_port = 'COM13'

# h5 file path
H5_FILE_PATH = '../../yumi-ubuntu-backup/sign_robot_control/h5_data/experiment/2021-3-21-processed/processed-wo.h5'

# yumi joint limits
yumi_left_lower_limits = np.array([-2.94, -2.50, -2.94, -2.15, -5.06, -1.53, -3.99])
yumi_left_upper_limits = np.array([ 2.94, 0.75, 2.94, 1.39, 5.06, 2.40, 3.99])
yumi_right_lower_limits = np.array([-2.94, -2.50, -2.94, -2.15, -5.06, -1.53, -3.99])
yumi_right_upper_limits = np.array([ 2.94, 0.75, 2.94, 1.39, 5.06, 2.40, 3.99])

# error threshold
threshold = 3

"""
create sensor message with specific joint angles
"""
def createSensorMsg(jointAngles):
    pSensorMessage = egm_pb2.EgmSensor()
    pSensorMessage.header.mtype = egm_pb2.EgmHeader.MessageType.MSGTYPE_CORRECTION
    pSensorMessage.header.seqno = 0
    pSensorMessage.header.tm = 0
    for i in range(6):
        pSensorMessage.planned.joints.joints.append(jointAngles[i])
    pSensorMessage.planned.externalJoints.joints.append(jointAngles[6])
    return pSensorMessage

"""
parse h5 file to joint angles
"""
def parse_h5(filename, group_name='group1'):
    f = h5py.File(filename, 'r')
    l_joint_angles = f[group_name+'/l_joint_angle_2'][:]
    r_joint_angles = f[group_name+'/r_joint_angle_2'][:]
    l_glove_angles = f[group_name+'/l_glove_angle_2'][:]
    r_glove_angles = f[group_name+'/r_glove_angle_2'][:]
    return l_joint_angles, r_joint_angles, l_glove_angles, r_glove_angles

"""
convert rad to degree
"""
def rad2degree(angle):
    return angle * 180.0 / np.pi

"""
clamp joint angles to limits
"""
def clamp_to_limits(joint_angle, lower_bound, upper_bound):
    return np.clip(joint_angle, lower_bound, upper_bound)

"""
switch joint angles
"""
def switch_joint_angle(joint_angle):
    print(joint_angle.shape)
    switch_angle = np.zeros_like(joint_angle)
    switch_angle[:, 0] = joint_angle[:, 0]
    switch_angle[:, 1] = joint_angle[:, 1]
    switch_angle[:, 6] = joint_angle[:, 2]
    switch_angle[:, 2] = joint_angle[:, 3]
    switch_angle[:, 3] = joint_angle[:, 4]
    switch_angle[:, 4] = joint_angle[:, 5]
    switch_angle[:, 5] = joint_angle[:, 6]
    return switch_angle

"""
parse received data
"""
def parse_data(data):
    pRobotMessage = egm_pb2.EgmRobot()
    pRobotMessage.ParseFromString(data)
    if pRobotMessage.HasField('header') and pRobotMessage.header.HasField('seqno') and pRobotMessage.header.HasField('tm') and pRobotMessage.header.HasField('mtype'):
        # print("SeqNo={} Tm={} Type={}".format(pRobotMessage.header.seqno, pRobotMessage.header.tm, pRobotMessage.header.mtype))
        # print(pRobotMessage.feedBack.joints.joints, pRobotMessage.feedBack.externalJoints.joints)
        joints = np.array(list(pRobotMessage.feedBack.joints.joints) + list(pRobotMessage.feedBack.externalJoints.joints))
    else:
        print("No header")
    return joints

"""
convert glove angles to electrical signal
"""
def convert_glove_angles(glove_angles):
    # Input *glove_angles* is of the size (50, 12)
    ndata = glove_angles.shape[0]
    elec_signal = np.zeros((ndata, 6), dtype="int32")
    optim_fourfin_range = [0, -1.6]
    optim_thumbroll_range = [0.1, 0.0]
    optim_thumbrot_range = [-1.0, 0.3] #[0.3, -1.0] # actually fixed at intermediate position when we still use S14 glove, since it doesn't measure this axis of motion
    max_elec = 2000
    min_elec = 0

    # conversion
    for i in range(ndata):
        # four fingers (for inspire hand electrical signal, order is: pinky->ring->middle->index)
        elec_signal[i, 3] = round((glove_angles[i, 0] - optim_fourfin_range[0]) / (optim_fourfin_range[1] - optim_fourfin_range[0]) * (max_elec - min_elec))
        elec_signal[i, 2] = round((glove_angles[i, 2] - optim_fourfin_range[0]) / (optim_fourfin_range[1] - optim_fourfin_range[0]) * (max_elec - min_elec))
        elec_signal[i, 1] = round((glove_angles[i, 4] - optim_fourfin_range[0]) / (optim_fourfin_range[1] - optim_fourfin_range[0]) * (max_elec - min_elec))
        elec_signal[i, 0] = round((glove_angles[i, 6] - optim_fourfin_range[0]) / (optim_fourfin_range[1] - optim_fourfin_range[0]) * (max_elec - min_elec))
        # thumb roll
        elec_signal[i, 4] = round((glove_angles[i, 9] - optim_thumbroll_range[0]) / (optim_thumbroll_range[1] - optim_thumbroll_range[0]) * (max_elec - min_elec))
        # thumb rot (actually fixed when using S14 generated data)
        elec_signal[i, 5] = round((glove_angles[i, 8] - optim_thumbrot_range[0]) / (optim_thumbrot_range[1] - optim_thumbrot_range[0]) * (max_elec - min_elec))

        # check if within range
        for j in range(6):
            if elec_signal[i, j] < 0 or elec_signal[i, j] > 2000:
                print("Error: Joint angle J{}={} of path point {} is out of bounds!".format(i, glove_angles[i, j], j))
                return None

    return elec_signal


if __name__ == '__main__':
    # socket for left arm
    sock_L = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    sock_L.bind((CLIENT_IP, CLIENT_PORT_L))
    # socket for right arm
    sock_R = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    sock_R.bind((CLIENT_IP, CLIENT_PORT_R))

    # inspire hands
    left_hand_controller = InspireHand(left_hand_port, 115200)
    right_hand_controller = InspireHand(right_hand_port, 115200)
    left_hand_controller.setspeed(1000, 1000, 1000, 1000, 1000, 1000)
    right_hand_controller.setspeed(1000, 1000, 1000, 1000, 1000, 1000)

    # parse h5 file
    l_joint_angles, r_joint_angles, l_glove_angles, r_glove_angles = parse_h5(H5_FILE_PATH)
    l_joint_angles = switch_joint_angle(rad2degree(clamp_to_limits(l_joint_angles, yumi_left_lower_limits, yumi_left_upper_limits)))
    r_joint_angles = switch_joint_angle(rad2degree(clamp_to_limits(r_joint_angles, yumi_right_lower_limits, yumi_right_upper_limits)))
    l_glove_signal = convert_glove_angles(l_glove_angles)
    r_glove_signal = convert_glove_angles(r_glove_angles)
    total_frames = len(l_joint_angles)
    t = 0
    done = False
    last_arm_command_time = -float('Inf')
    last_hand_command_time = -float('Inf')
    last_verbose_time = -float('Inf')

    while True:
        # receive data
        try:
            # left arm
            data_L, server_L = sock_L.recvfrom(1024) # buffer size is 1024 bytes
            # right arm
            data_R, server_R = sock_R.recvfrom(1024) # buffer size is 1024 bytes
        except:
            pass

        # check if motion is done
        current_l_joints = parse_data(data_L)
        current_r_joints = parse_data(data_R)
        error = np.concatenate([current_l_joints, current_r_joints]) - np.concatenate([l_joint_angles[t], r_joint_angles[t]])
        done = (error < threshold).all()
        if time.time() - last_verbose_time > 0.5:
            print('t', t, 'done', done)
            print('current angles', np.concatenate([current_l_joints, current_r_joints]))
            print('target angles', np.concatenate([l_joint_angles[t], r_joint_angles[t]]))
            last_verbose_time = time.time()

        # next frame
        if done:
            t += 1
            if t == total_frames:
                t = 0
        # send control command not too fast
        elif time.time() - last_arm_command_time > 0.005:
            # left arm
            pSensorMessage = createSensorMsg(l_joint_angles[t])
            sock_L.sendto(pSensorMessage.SerializeToString(), server_L)
            # right arm
            pSensorMessage = createSensorMsg(r_joint_angles[t])
            sock_R.sendto(pSensorMessage.SerializeToString(), server_R)
            last_arm_command_time = time.time()

        # set inspire hand pos
        if time.time() - last_hand_command_time > 0.005:
            left_hand_controller.setpos(l_glove_signal[t, 0], l_glove_signal[t, 1], l_glove_signal[t, 2], l_glove_signal[t, 3], l_glove_signal[t, 4], l_glove_signal[t, 5])
            right_hand_controller.setpos(r_glove_signal[t, 0], r_glove_signal[t, 1], r_glove_signal[t, 2], r_glove_signal[t, 3], r_glove_signal[t, 4], r_glove_signal[t, 5])
            last_hand_command_time = time.time()
