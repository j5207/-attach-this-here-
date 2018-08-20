# coding:utf-8
import socket

def netsend(a, localhost, port):
    for i in range(len(a)):
        client = socket.socket()  # 默认是AF_INET、SOCK_STREAM
        client.connect((localhost, port))
        line = str(a[i])
        print(line)
        client.send(line.encode("utf-8"))
        client.close()

