import serial
from time import sleep
import sys
import socket
import time

HOST_IP = "192.168.43.35"
HOST_PORT = 8888


COM_PORT = 'COM8'  # 請自行修改序列埠名稱
BAUD_RATES = 9600
ser = serial.Serial(COM_PORT, BAUD_RATES, timeout = 5) #5秒內沒有傳回訊息將回傳false

print("Starting socket: TCP...")

#1.create socket object:socket=socket.socket(family,type)
socket_tcp = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
print("TCP server listen @ %s:%d!" %(HOST_IP, HOST_PORT) )
host_addr = (HOST_IP, HOST_PORT)

#2.bind socket to addr:socket.bind(address)
socket_tcp.bind(host_addr)

#3.listen connection request:socket.listen(backlog)
socket_tcp.listen(1)

#4.wait for client:connection,address=socket.accept()
socket_con, (client_ip, client_port) = socket_tcp.accept()
print("Connection accepted from %s." %client_ip)
socket_con.send("Welcome to RPi TCP server!")

try:
    data=socket_con.recv(512)
    while True:
        try:
            if len(data)>0:
                print("Received:%s"%data)
                if data== 'f':
                    print('forward')
                    ser.write(b'f\n')  # 訊息必須是位元組類型
                    sleep(0.5)              # 暫停0.5秒，再執行底下接收回應訊息的迴圈
                elif data== 'b':
                    print('backward')
                    ser.write(b'b\n')
                    sleep(0.5)
                elif data== 'l':
                    print('left shift')
                    ser.write(b'l\n')
                    sleep(0.5)
                elif data== 'r':
                    print('right shift')
                    ser.write(b'r\n')
                    sleep(0.5)
                elif data== '1':
                    print('left spin')
                    ser.write(b'1\n')
                    sleep(0.5)
                elif data== '2':
                    print('right spin')
                    ser.write(b'2\n')
                    sleep(0.5)
                elif data == 'h':
                    print('halt')
                    ser.write(b'h\n')
                    sleep(0.5)
                elif data == 'e':
                    ser.close()
                    print('exit')
                    sys.exit()
                    
                socket_con.send(data) #傳回給client相同訊息已示收到
                time.sleep(1)
                while ser.in_waiting:
                    mcu_feedback = ser.readline().decode()  # 接收arduino回應訊息並解碼
                    print('控制板回應：', mcu_feedback)
                    break
                
                continue
        
        except KeyboardInterrupt:
            ser.close()
            print('再見！')
        
except Exception:
    socket_tcp.close()
    sys.exit(1)
            

