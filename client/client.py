import socket
import time
import egm_pb2
import h5py
import numpy as np

# IP address & ports
CLIENT_IP = "192.168.125.10"
CLIENT_PORT_L = 6510
CLIENT_PORT_R = 6511

# h5 file path
H5_FILE_PATH = '../../yumi-ubuntu-backup/sign_robot_control/h5_data/experiment/2021-3-21-processed/processed-wo.h5'

# yumi joint limits
yumi_left_lower_limits = np.array([-2.94, -2.50, -2.94, -2.15, -5.06, -1.53, -3.99])
yumi_left_upper_limits = np.array([ 2.94, 0.75, 2.94, 1.39, 5.06, 2.40, 3.99])
yumi_right_lower_limits = np.array([-2.94, -2.50, -2.94, -2.15, -5.06, -1.53, -3.99])
yumi_right_upper_limits = np.array([ 2.94, 0.75, 2.94, 1.39, 5.06, 2.40, 3.99])


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

if __name__ == '__main__':
    # socket for left arm
    sock_L = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    sock_L.bind((CLIENT_IP, CLIENT_PORT_L))
    # socket for right arm
    sock_R = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    sock_R.bind((CLIENT_IP, CLIENT_PORT_R))

    # parse h5 file
    l_joint_angles, r_joint_angles, l_glove_angles, r_glove_angles = parse_h5(H5_FILE_PATH)
    l_joint_angles = switch_joint_angle(rad2degree(clamp_to_limits(l_joint_angles, yumi_left_lower_limits, yumi_left_upper_limits)))
    r_joint_angles = switch_joint_angle(rad2degree(clamp_to_limits(r_joint_angles, yumi_right_lower_limits, yumi_right_upper_limits)))
    total_frames = len(l_joint_angles)
    t = 0

    while True:
        # left arm
        data_L, server_L = sock_L.recvfrom(1024) # buffer size is 1024 bytes
        print("received message from ", server_L)
        # pRobotMessage = egm_pb2.EgmRobot()
        # pRobotMessage.ParseFromString(data)
        # if pRobotMessage.HasField('header') and pRobotMessage.header.HasField('seqno') and pRobotMessage.header.HasField('tm') and pRobotMessage.header.HasField('mtype'):
        #     print("SeqNo={} Tm={} Type={}".format(pRobotMessage.header.seqno, pRobotMessage.header.tm, pRobotMessage.header.mtype))
        #     print(pRobotMessage.feedBack.joints.joints, pRobotMessage.feedBack.externalJoints.joints)
        # else:
        #     print("No header")
        jointAngles_L = l_joint_angles[t]  # [-30, -90, 0, 0, 0, 0, 60]
        pSensorMessage = createSensorMsg(jointAngles_L)
        sock_L.sendto(pSensorMessage.SerializeToString(), server_L)

        # right arm
        data_R, server_R = sock_R.recvfrom(1024) # buffer size is 1024 bytes
        print("received message from ", server_R)
        jointAngles_R = r_joint_angles[t]  # [30, -90, 0, 0, 0, 0, -60]
        pSensorMessage = createSensorMsg(jointAngles_R)
        sock_R.sendto(pSensorMessage.SerializeToString(), server_R)

        time.sleep(0.2)
        t += 1
        if t == total_frames:
            t = 0
