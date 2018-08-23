# coding:utf-8
import socket

def netsend(a, host, port):
    for i in range(len(a)):
        client = socket.socket()  # 默认是AF_INET、SOCK_STREAM
        client.connect((host, port))
        line = str(a[i])
        print(line)
        client.send(line.encode("utf-8"))
        client.close()

netsend([100,200,-1], '192.168.1.115', 6868)