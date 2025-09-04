import socket
import threading
import numpy as np
from dynamixel_sdk import *
from dynamixel_motors import * # Motor에 따라 설정 수행

# XM430-2개 / XC330 5개 액츄에이터 제어 코드

test_message = ''

UDP_IP = "127.0.0.1"  # 들어오는 UDP 신호의 IP 주소
UDP_PORT = 9001  # 들어오는 UDP 신호의 포트 번호

Target_IP = "127.0.0.1" # 루프백, 유니티
Target_PORT = 9002

# UDP 소켓 설정
sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
sock.bind((UDP_IP, UDP_PORT))

BAUDRATE = 1000000
DEVICENAME = ('COM10')  # 포트 번호 (시스템에 따라 변경 가능)
# 다이나믹셀 포트 초기화
portHandler = PortHandler(DEVICENAME)
packetHandler = PacketHandler(2.0)  # 다이나믹셀 프로토콜 버전

# 포트 열기
if not portHandler.openPort():
    print("Failed to open the port")
    quit()

# 포트 설정
if not portHandler.setBaudRate(BAUDRATE):
    print("Failed to change the baudrate")
    quit()

# BulkWrite 초기화
groupBulkWrite = GroupBulkWrite(portHandler, packetHandler)
groupBulkWrite.clearParam()

# BulkRead 초기화
groupBulkRead = GroupBulkRead(portHandler, packetHandler)
groupBulkRead.clearParam()

def Initialize_Motors():
    for id,type in Actuators.items():
        if type == "XM430":
            Motors.append(XM430(id,portHandler,packetHandler,groupBulkRead,groupBulkWrite))
        elif type == "XC330":
            Motors.append(XC330(id,portHandler,packetHandler,groupBulkRead,groupBulkWrite))
        else:
            print("Invalid Type is required")
        MotorsinitialAngle[id] = f"{Motors[id].initialPosition * 360 / 4095:.2f}"    

# ask 받는 코드 추가 필요할듯######################
def sendIntialAngle():
    SendMsgToUnity(",".join(MotorsinitialAngle))

def UpdateMotorStatus():
    while True:
        try:
            sendMotorData()
            receiveMotorData()
        except Exception as e:
            print(e)
            pass

def sendMotorData():
    groupBulkWrite.clearParam()
    CheckChangeModes()
    for i in range(len(Motors)):
        Motors[i].sendMotorData()
    groupBulkWrite.txPacket()

def CheckChangeModes():
    for i in range(len(Motors)):
        if(GoalTorqueModes[i] != 10):
            Motors[i].enableTorque(GoalTorqueModes[i])
            GoalTorqueModes[i] = 10
            #print("Goal Torque 변경")
        if (GoalOperatingModes[i] != 10):
            Motors[i].enableTorque(0)
            Motors[i].changeOperatingMode(GoalOperatingModes[i])
            Motors[i].enableTorque(1)
            """
            if(GoalOperatingModes[i] == 0):
                print("Goal Opearting Mode : Current control 변경")
            elif(GoalOperatingModes[i] == 5):
                print("Goal Opearting Mode : Current-based Position Control 변경")
            """
            GoalOperatingModes[i] = 10

def SendMsgToUnity(msg : str):
    sock.sendto(bytes(msg, 'utf-8'), (Target_IP, Target_PORT))

def receiveMotorData():
    groupBulkRead.txRxPacket()
    for i in range(len(Motors)):
        result = Motors[i].receiveMotorData()
        MotorsPresentTorques[i] = result[0]
        MotorsCurrentPositions[i] = result[1]
        msg = f"{i},{MotorsCurrentPositions[i]},{MotorsPresentTorques[i]}"
        SendMsgToUnity(msg)

def ReceiveUdpData():
    while(True):
        try:
            message, addr = sock.recvfrom(1024)  # 버퍼 크기 1024
            ParsingData(message)
        except Exception as e:
            #print(e)
            pass

def ParsingData(message : bytes):
    write_data = message.decode()
    #print(write_data)
    write_datas = write_data.split(",")
    if (write_datas[0] == 'p'):
        i = int(write_datas[1])
        goal_position = float(write_datas[2])
        goal_torque = float(write_datas[3])
        Motors[i].updateGoalPosition(goal_position)
        Motors[i].updateGoalTorque(goal_torque)
        #print(write_data)

    elif (write_datas[0] == 'c'):
        i = int(write_datas[1])
        goal_torque = float(write_datas[2])
        Motors[i].updateGoalTorque(goal_torque)
        #print(f"goal_current 변경,{goal_current}")

    elif (write_datas[0] == 't'):
        id = int(write_datas[1])
        mode = int(write_datas[2])
        enableTorque(id, mode)

    elif (write_datas[0] == 'co'):  # Change Operating Mode
        id = int(write_datas[1])
        mode = int(write_datas[2])
        changeOperatingMode(id, mode)

    else:
        print("Invalid Data")

def enableTorque(id,mode):
    GoalTorqueModes[id] = int(mode)

def changeOperatingMode(id, mode):
    GoalOperatingModes[id] = int(mode)

def TestCommand():
    while(True):
        try:
            msg = input(">>>")
            write_datas = msg.split(',')
            if (write_datas[0] == 'p'):
                i = int(write_datas[1])
                goal_position = float(write_datas[2])
                goal_torque = float(write_datas[3])
                Motors[i].updateGoalPosition(goal_position)
                Motors[i].updateGoalTorque(goal_torque)
                print("hi")

            elif (write_datas[0] == 'c'):
                i = int(write_datas[1])
                goal_torque = float(write_datas[2])
                Motors[i].updateGoalTorque(goal_torque)

            elif (write_datas[0] == 't'):
                id = int(write_datas[1])
                mode = int(write_datas[2])
                enableTorque(id, mode)

            elif (write_datas[0] == 'co'):  # Change Operating Mode
                id = int(write_datas[1])
                mode = int(write_datas[2])
                changeOperatingMode(id, mode)

        except Exception as e:
            print(e)

def Report( period:float = 0.1):
    while(True):
        time.sleep(period)
        for i in range(len(Motors)):
            print(f"{i},{MotorsCurrentPositions[i]},{MotorsPresentTorques[i]}")

Actuators = {0:"XC330",1:"XC330",2:"XC330",3:"XC330",4:"XC330",5:"XC330",6:"XC330",7:"XC330",8:"XC330",9:"XC330",10:"XC330",11:"XC330"}
Motors = []
MotorsinitialAngle=[0 for i in range(len(Actuators))]
MotorsCurrentPositions=[0 for i in range(len(Actuators))]
MotorsPresentTorques=[0 for i in range(len(Actuators))]
MotorsGoalPositions=[0 for i in range(len(Actuators))]
MotorsGoalTorques=[100 for i in range(len(Actuators))]
GoalOperatingModes = [10 for i in range(len(Actuators))]
GoalTorqueModes = [10 for i in range(len(Actuators))]
Initialize_Motors()
sendIntialAngle()

if __name__ == "__main__":
    t1 = threading.Thread(target=UpdateMotorStatus)
    t1.start()

    t2 = threading.Thread(target=ReceiveUdpData)
    t2.start()

    t3 = threading.Thread(target=TestCommand)
    t3.start()

    # t4 = threading.Thread(target=Report, args=(1,))
    # t4.start()

    try:
        print("시작")
        while(True):
            time.sleep(10)
    except Exception as e:
        print(e)

    finally:
        print("종료")
        MotorsGoalCurrents = [0 for i in range(len(Motors))]
        for i in range(len(Motors)):
            Motors[i].updateGoalPosition(MotorsGoalPositions[i])
            Motors[i].updateGoalTorque(MotorsGoalTorques[i])
        sendMotorData()



