# import socket
# HOST = '127.0.0.1'
# PORT = 8000

# server = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
# server.bind((HOST, PORT))
# server.listen(10)

# while True:
#     conn ,address = server.accept()
#     clientMessage = str(conn.recv(1024), encoding="utf-8")
    
#     print('Client message is:', clientMessage)
    
#     serverMessage = 'I\'m here!'
#     conn.sendall(serverMessage.encode())
#     conn.close()

# -*- coding: utf-8 -*-
# import json
# import socket

# # HOST & PORT
# # with open('HOST_PORT.json', 'r', encoding='utf-8') as f:
# #     HOST_PORT = json.load(f)

# HOST = '127.0.0.1'
# PORT = 8000

# server = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
# server.bind((HOST, PORT))

# server.listen(5)


# while True:
#     conn, addr = server.accept()
#     clientMessage = str(conn.recv(1024), encoding='utf-8')

#     print('Client message is:', clientMessage)

#     # Repeat
#     serverMessage = clientMessage
#     conn.sendall(serverMessage.encode())
#     conn.close()


import socket
import time
import sys
# import pyrealsense2 as rs


#define host ip: Rpi's IP
HOST_IP = "192.168.43.194"
HOST_PORT = 8888
print("Starting socket: TCP...")
#1.create socket object:socket=socket.socket(family,type)
socket_tcp = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
print("TCP server listen @ %s:%d!" %(HOST_IP, HOST_PORT) )
host_addr = (HOST_IP, HOST_PORT)
#2.bind socket to addr:socket.bind(address)
socket_tcp.bind(host_addr)
#3.listen connection request:socket.listen(backlog)
socket_tcp.listen(1)
#4.waite for client:connection,address=socket.accept()
socket_con, (client_ip, client_port) = socket_tcp.accept()
print("Connection accepted from %s." %client_ip)
socket_con.send(b"Welcome to RPi TCP server!")


while True:
    try:
        data=socket_con.recv(4096)
        if len(data)>0:
            print("Received:%s"%(data.decode('utf-8')))
#             if data=='stop':
# 			    stop();
#             elif data=='gogo':
# 			    gogo();
            socket_con.send(b'success!')
            time.sleep(0.01)
            continue
    except Exception as e:
            print(e)
            socket_tcp.close()
            sys.exit(1)