import socket
import time
import egm_pb2

# IP address & ports
CLIENT_IP = "192.168.125.10"
CLIENT_PORT_L = 6510
CLIENT_PORT_R = 6511

# socket for left arm
sock_L = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
sock_L.bind((CLIENT_IP, CLIENT_PORT_L))
# socket for right arm
sock_R = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
sock_R.bind((CLIENT_IP, CLIENT_PORT_R))


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


if __name__ == '__main__':
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
        jointAngles_L = [-30, -90, 0, 0, 0, 0, 60]
        pSensorMessage = createSensorMsg(jointAngles_L)
        sock_L.sendto(pSensorMessage.SerializeToString(), server_L)

        # right arm
        data_R, server_R = sock_R.recvfrom(1024) # buffer size is 1024 bytes
        print("received message from ", server_R)
        jointAngles_R = [30, -90, 0, 0, 0, 0, -60]
        pSensorMessage = createSensorMsg(jointAngles_R)
        sock_R.sendto(pSensorMessage.SerializeToString(), server_R)

        time.sleep(0.1)
