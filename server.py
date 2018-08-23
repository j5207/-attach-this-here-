# coding:utf-8
import socket
import cv2
import numpy as np

#生成一个空灰度图像


server = socket.socket()    # 默认是AF_INET、SOCK_STREAM
server.bind(('192.168.1.115',6868))     # 将主机号与端口绑定到套接字
server.listen()     # 设置并启动TCP监听器
pos = []
while True:
    conn,addr = server.accept()     # 被动接受TCP连接，一直等待连接到达
    print(addr)
    data = conn.recv(10)      # 接收TCP消息，并制定最大长度
    coor = str(data)
    print(coor)
    coor = coor[2:-1]
    coor = int(coor)
    if coor == -1:
        print(pos)
        img = np.zeros((480, 640, 3), np.uint8)
        for i in range(int(len(pos) / 2)):
            cv2.circle(img, (pos[2*i], pos[2*i + 1]), 20, (0, 0, 255), -1)  # 修改最后一个参数
        cv2.namedWindow("projector", cv2.WND_PROP_FULLSCREEN)
        cv2.setWindowProperty("projector", cv2.WND_PROP_FULLSCREEN, cv2.WINDOW_FULLSCREEN)
        cv2.imshow("projector", img)
        cv2.waitKey(3000)
        cv2.destroyAllWindows()
        pos = []
    else:
        pos.append(coor)




