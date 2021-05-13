import socket
import egm_pb2

CLIENT_IP = "192.168.125.10"
CLIENT_PORT = 6510

sock = socket.socket(socket.AF_INET, # Internet
                     socket.SOCK_DGRAM) # UDP
sock.bind((CLIENT_IP, CLIENT_PORT))

while True:
    data, server_addr = sock.recvfrom(1024) # buffer size is 1024 bytes
    print("received message from ", server_addr)
    pRobotMessage = egm_pb2.EgmRobot()
    pRobotMessage.ParseFromString(data)
    if pRobotMessage.HasField('header') and pRobotMessage.header.HasField('seqno') and pRobotMessage.header.HasField('tm') and pRobotMessage.header.HasField('mtype'):
        print("SeqNo={} Tm={} Type={}".format(pRobotMessage.header.seqno, pRobotMessage.header.tm, pRobotMessage.header.mtype))
        print(pRobotMessage.feedBack.joints.joints, pRobotMessage.feedBack.externalJoints.joints)
    else:
        print("No header")

    pSensorMessage = egm_pb2.EgmSensor()
    pSensorMessage.header.mtype = egm_pb2.EgmHeader.MessageType.MSGTYPE_CORRECTION
    pSensorMessage.header.seqno = 0
    pSensorMessage.header.tm = 0
    jointAngles = [0, 0, 0, 0, 0, 0, 10]
    for i in range(6):
        pSensorMessage.planned.joints.joints.append(jointAngles[i])
    pSensorMessage.planned.externalJoints.joints.append(jointAngles[6])
    sock.sendto(pSensorMessage.SerializeToString(), server_addr)