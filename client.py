


# import socket
# import time
# import sys
# import pyrealsense2 as rs

# #RPi's IP
# SERVER_IP = "192.168.43.194"
# SERVER_PORT = 8888

# # Create a context object. This object owns the handles to all connected realsense devices
# pipeline = rs.pipeline()
# pipeline.start()

# print("Starting socket: TCP...")
# server_addr = (SERVER_IP, SERVER_PORT)
# socket_tcp = socket.socket(socket.AF_INET, socket.SOCK_STREAM)

# while True:
#     try:
#         print("Connecting to server @ %s:%d..." %(SERVER_IP, SERVER_PORT))
#         socket_tcp.connect(server_addr)
#         break
#     except Exception:
#         print("Can't connect to server,try it latter!")
#         time.sleep(1)
#         continue
# # print("Please input gogo or stop to turn on/off the motor!")
# while True:
#     try:
#         frames = pipeline.wait_for_frames()
#         depth = frames.get_depth_frame()
#         if not depth: continue
    
#         coverage = [0]*64
#         for y in range(480):
#             for x in range(640):
#                 dist = depth.get_distance(x, y)
#                 if 0 < dist and dist < 1:
#                     coverage[x//10] += 1

#             if y%20 is 19:
#                 line = ""
#                 for c in coverage:
#                     line += " .:45678W"[c//25]
#                 coverage = [0]*64
                
#                 # print(line)
                
#                 data = socket_tcp.recv(4096)
#                 if len(data)>0:
#                     print("Received: %s" % data)
#                     socket_tcp.send(bytes(line,'utf-8'))
#                     time.sleep(0.01)
#                     continue
                
    
        
#     except Exception as e:
#         print(e)
#         socket_tcp.close()
#         socket_tcp=None
#         sys.exit(1)

import socket
import time
import sys
#RPi's IP
SERVER_IP = "192.168.43.35"
SERVER_PORT = 8888


print("Starting socket: TCP...")
server_addr = (SERVER_IP, SERVER_PORT)
socket_tcp = socket.socket(socket.AF_INET, socket.SOCK_STREAM)

while True:
    try:
        print("Connecting to server @ %s:%d..." %(SERVER_IP, SERVER_PORT))
        socket_tcp.connect(server_addr)
        break
    except Exception:
        print("Can't connect to server,try it latter!")
        time.sleep(1)
        continue
# print("Please input gogo or stop to turn on/off the motor!")
while True:
    try:
        data = socket_tcp.recv(512)
        if len(data)>0:
            print("Received: %s" % data.decode())
            command=input()
            socket_tcp.send(command.encode())
            time.sleep(0.01)
            continue
    except Exception as e:
        print(e)
        socket_tcp.close()
        socket_tcp=None
        sys.exit(1)