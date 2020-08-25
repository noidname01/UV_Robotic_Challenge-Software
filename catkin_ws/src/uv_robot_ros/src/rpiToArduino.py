#!/udsr/bin/env python2.7
import serial
from time import sleep
import sys
import socket
import time

import rospy
from uv_robot_ros.srv import cmdToRpi, cmdToRpiResponse
#HOST_IP = "192.168.0.203"
#HOST_IP = "0.0.0.0"
#HOST_PORT = 8888


COM_PORT = '/dev/ttyUSB0'  # 請自行修改序列埠名稱
BAUD_RATES = 9600
ser = serial.Serial(COM_PORT, BAUD_RATES, timeout = 5) #5秒內沒有傳回訊息將回傳false

def handle_cmd_to_arduino(req):

    data = req.cmdType+str(req.dist_or_deg) #action type and params
    ser.write(data.encode()) #encode and send to Arduino
    
    res = cmdToRpiResponse() #create a response format
    recv = ser.readline().strip() #get message from Arduino
    if recv == "c":
        """
        if it receives "c", means that it work correctly
        """
        res.isComplete = True
        res.errorMsg = ""
    elif recv == "e":
        """
        if it receives "e", means that TOF sensored
        that obstacle is too close
        """
        res.isComplete = False
        res.errorMsg = "Too close or too far"
    elif recv == "h":
        """
        if it receives "h", means that human detector find human
        """
        res.isComplete = False
        res.errorMsg = "Human passing by"

    return res


def cmd_to_rpi_server():
    rospy.init_node('rpi_to_arduino')
    s = rospy.Service('cmdToRpiService', cmdToRpi, handle_cmd_to_arduino)
    rospy.spin()
    
if __name__ == "__main__":
    cmd_to_rpi_server()
#print("Starting socket: TCP...")

#1.create socket object:socket=socket.socket(family,type)
#socket_tcp = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
#print("TCP server listen @ %s:%d!" %(HOST_IP, HOST_PORT) )
#host_addr = (HOST_IP, HOST_PORT)

#2.bind socket to addr:socket.bind(address)
#socket_tcp.bind(host_addr)

#3.listen connection request:socket.listen(backlog)
#socket_tcp.listen(1)

#4.wait for client:connection,address=socket.accept()
#socket_con, (client_ip, client_port) = socket_tcp.accept()
#print("Connection accepted from %s." %client_ip)
#socket_con.send(b"Welcome to RPi TCP server!")

#try:
#    while True:
#        try:
#            data = socket_con.recv(512)
#            data = data.decode()
#            if len(data)>0:
#                print("Received:%s"%data)
                '''
                if data== 'f':
                    print('forward')
                    ser.write(b'f')  # 訊息必須是位元組類型
                    #sleep(0.5)              # 暫停0.5秒，再執行底下接收回應訊息的迴圈
                elif data== 'b':
                    print('backward')
                    ser.write(b'b')
                    #sleep(0.5)
                elif data== 'l':
                    print('left spin')
                    ser.write(b'l')
                    #sleep(0.5)
                elif data== 'r':
                    print('right spin')
                    ser.write(b'r')
                    #sleep(0.5)
                elif data== 'm':
                    print('left shift')
                    ser.write(b'm')
                elif data== 'n':
                    print('right shift')
                    ser.write(b'n')
                elif data== '1':
                    #print('left spin')
                    print('adjust 1')
                    ser.write(b'1')
                    #sleep(0.5)
                elif data== '2':
                    #print('right spin')
                    print('adjust 2')
                    ser.write(b'2')
                    #sleep(0.5)
                elif data== '3':
                    #print('left spin')
                    print('adjust 3')
                    ser.write(b'3')
                    #sleep(0.5)
                elif data== '4':
                    #print('right spin')
                    print('adjust 4')
                    ser.write(b'4')
                    #sleep(0.5)
                elif data == 'h':
                    print('halt')
                    ser.write(b'h')
                    #sleep(0.5)
                elif data == 'e':
                    ser.close()
                    print('exit')
                    sys.exit()
                else:
                    data = "Not Recognized"
                '''
#               ser.write(data.encode())
#               socket_con.send(data.encode()) #傳回給client相同訊息已示收到
                #time.sleep(1)
                
#               recv = ser.readline().strip()
#               while recv:
#                   print(recv)
#                   recv = ""
#                
#        except KeyboardInterrupt:
#            ser.close()
#            print('再見！')
#            sys.exit(1)
        
#except Exception as e:
#   print(e)
#    socket_tcp.close()
#    sys.exit(1)
            

