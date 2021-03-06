#!/usr/bin/env python
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


COM_PORT = '/dev/tty54' 
BAUD_RATES = 9600
ser = serial.Serial(COM_PORT, BAUD_RATES, timeout = 5) 

def handle_cmd_to_arduino(req):

    data = req.cmdType+str(req.dist_or_deg) #action type and params
    ser.write(data.encode()) #encode and send to Arduino

    if (req.dist_or_deg == "0"):
        res.isComplete = True
        res.errorMsg = "No Error!"
        return res

    
    res = cmdToRpiResponse() #create a response format
    recv = ser.readline().strip() #get message from Arduino
    if recv == "c":
        """
        if it receives "c", means that it work correctly
        """
        res.isComplete = True
        res.errorMsg = ""
    elif recv == "el":
        """
        if it receives "el", means that TOF sensored
        that obstacle is too close
        """
        res.isComplete = False
        res.errorMsg = "Left hand side has something block the way!"
    elif recv == "er":
        """
        if it receives "er", means that right hand side has something blocks.
        """
        res,isComplete = False
        res.errorMsg = "Right hand side has something block the way!"


    elif recv == "human":
        """
        if it receives "h", means that human detector find human
        """
        res.isComplete = False
        res.errorMsg = "Human passing by"

    elif recv == "resume":
        """
        if it receives "resume", means that no human detect and can continue
        """
        res.isComplete = True
        res.errorMsg = "Human Left"


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
#               ser.write(data.encode())
#               socket_con.send(data.encode()) 
                #time.sleep(1)
                
#               recv = ser.readline().strip()
#               while recv:
#                   print(recv)
#                   recv = ""
#                
#        except KeyboardInterrupt:
#            ser.close()
#            print("bye")
#            sys.exit(1)
        
#except Exception as e:
#   print(e)
#    socket_tcp.close()
#    sys.exit(1)
            

