# coding:utf-8
import socket
import numpy as np
import time
import cv2

pick_start = False


def coor_tran(pos):
    for i in range(int(len(pos) / 2)):
        pos[2 * i] = int( pos[2 * i] * 1600 / 640 )
        pos[2 * i + 1] = int( pos[2 * i + 1] * 900 / 480 )
    return pos

def draw_x(img, x, y, line):
    cv2.line(img, (x + int(line/2), y + int(line/2)),(x - int(line/2), y - int(line/2)), (255, 255, 255), 10)
    cv2.line(img, (x - int(line / 2), y + int(line / 2)), (x + int(line / 2), y - int(line / 2)), (255, 255, 255), 10)
    return img

def draw_arrow(img, x1, y1, x2, y2, line):
    cv2.arrowedLine(img, (int(0.8*x1 + 0.2*x2), int(0.8*y1 + 0.2*y2)), (int(0.2*x1 + 0.8*x2), int(0.2*y1+0.8*y2)),(255, 255, 255), line)
    return img

def intro():
    img = np.zeros((900, 1600, 3), np.uint8) * 255
    cv2.circle(img, (int(1600*428/640),int(207*900/480)), 40, (255, 255, 255), -1)
    img = draw_x(img,int(325*1600/640),int(420*900/480), 40)
    img = draw_arrow(img, int(1600*428/640), int(207*900/480), int(1600*325/640),int(420*900/480),5)
    cv2.namedWindow("projector", cv2.WND_PROP_FULLSCREEN)
    cv2.setWindowProperty("projector", cv2.WND_PROP_FULLSCREEN, cv2.WINDOW_FULLSCREEN)
    cv2.imshow("projector", img)
    cv2.waitKey(0)


    img = np.zeros((900, 1600, 3), np.uint8) * 255
    cv2.circle(img, (int(1600*428/640),int(207*900/480)), 40, (255, 255, 255), -1)
    cv2.circle(img, (int(1600*428/640+80),int(207*900/480+80)), 40, (255, 255, 255), -1)
    img = draw_x(img,int(207*1600/640),int(207*900/480), 40)
    img = draw_arrow(img, int(1600*428/640+40), int(207*900/480+40), int(1600*207/640),int(207*900/480),5)
    cv2.namedWindow("projector", cv2.WND_PROP_FULLSCREEN)
    cv2.setWindowProperty("projector", cv2.WND_PROP_FULLSCREEN, cv2.WINDOW_FULLSCREEN)
    cv2.imshow("projector", img)
    cv2.waitKey(0)

    img = np.zeros((900, 1600, 3), np.uint8) * 255
    cv2.circle(img, (int(1600*428/640),int(207*900/480)), 100, (255, 255, 255), 5)
    img = draw_x(img,int(207*1600/640),int(207*900/480), 40)
    img = draw_arrow(img, int(1600*428/640), int(207*900/480), int(1600*207/640),int(207*900/480),5)
    cv2.namedWindow("projector", cv2.WND_PROP_FULLSCREEN)
    cv2.setWindowProperty("projector", cv2.WND_PROP_FULLSCREEN, cv2.WINDOW_FULLSCREEN)
    cv2.imshow("projector", img)
    cv2.waitKey(0)

    img = np.zeros((900, 1600, 3), np.uint8) * 255
    cv2.circle(img, (int(1600*200/640),int(400*900/480)), 70, (255, 255, 255), 5)
    img = draw_x(img,int(207*1600/640),int(207*900/480), 40)
    img = draw_arrow(img, int(1600*200/640), int(400*900/480), int(1600*207/640),int(207*900/480),5)
    cv2.namedWindow("projector", cv2.WND_PROP_FULLSCREEN)
    cv2.setWindowProperty("projector", cv2.WND_PROP_FULLSCREEN, cv2.WINDOW_FULLSCREEN)
    cv2.imshow("projector", img)
    cv2.waitKey(0)

    img = np.zeros((900, 1600, 3), np.uint8) * 255
    cv2.circle(img, (int(1600*500/640),int(400*900/480)), 40, (255, 255, 255), -1)
    cv2.circle(img, (int(1600*500/640+80),int(400*900/480)), 40, (255, 255, 255), -1)
    img = draw_x(img,int(207*1600/640),int(207*900/480), 40)
    img = draw_arrow(img, int(1600*500/640+40), int(400*900/480), int(1600*207/640),int(207*900/480),5)
    cv2.namedWindow("projector", cv2.WND_PROP_FULLSCREEN)
    cv2.setWindowProperty("projector", cv2.WND_PROP_FULLSCREEN, cv2.WINDOW_FULLSCREEN)
    cv2.imshow("projector", img)
    cv2.waitKey(0)

    img = np.zeros((900, 1600, 3), np.uint8) * 255
    cv2.namedWindow("projector", cv2.WND_PROP_FULLSCREEN)
    cv2.setWindowProperty("projector", cv2.WND_PROP_FULLSCREEN, cv2.WINDOW_FULLSCREEN)
    cv2.imshow("projector", img)
    cv2.waitKey(0)



#intro()
server = socket.socket()    # 默认是AF_INET、SOCK_STREAM
server.bind(('10.194.55.236',6868))     # 将主机号与端口绑定到套接字
server.listen(999999999)     # 设置并启动TCP监听器
pos = []
img = np.zeros((900, 1600, 3), np.uint8) * 255
cv2.namedWindow("projector", cv2.WND_PROP_FULLSCREEN)
cv2.setWindowProperty("projector", cv2.WND_PROP_FULLSCREEN, cv2.WINDOW_FULLSCREEN)
cv2.imshow("projector", img)
cv2.waitKey(20)
while True:
    k = cv2.waitKey(1) & 0xFF  # large wait time to remove freezing
    if k == 113 or k == 27:
        break
    conn,addr = server.accept()     # 被动接受TCP连接，一直等待连接到达
    data = conn.recv(10)      # 接收TCP消息，并制定最大长度
    coor = str(data)
    print(coor)
    #coor = coor[2:-1]
    coor = int(coor)
    if coor == -1 or coor == 1 or coor == 2 or coor == 0 or coor == 8:
        pos = coor_tran(pos)
        print(pos)
        if pos is not None:
            if coor == -1:
                if not pick_start:
                    img = np.zeros((900, 1600, 3), np.uint8)
                    for i in range(int(len(pos) / 2)):
                        cv2.circle(img, (pos[2*i] + 20, pos[2*i + 1]), 40, (255, 255, 255), -1)  # 修改最后一个参数
                    # cv2.namedWindow("projector", cv2.WND_PROP_FULLSCREEN)
                    # cv2.setWindowProperty("projector", cv2.WND_PROP_FULLSCREEN, cv2.WINDOW_FULLSCREEN)
                    cv2.imshow("projector", img)
                    #cv2.waitKey(1)
                pos = []

            elif coor == 1:
                pick_start = True
                img = np.zeros((900, 1600, 3), np.uint8)
                for i in range(int(len(pos) / 2)):
                    cv2.rectangle(img, (pos[2*i]-30, pos[2*i + 1]-30), (pos[2*i]+30, pos[2*i+1]+ 30), (255, 255, 255), -1)
                    #cv2.circle(img, (pos[2*i], pos[2*i + 1]), 35, (0, 0, 255), -1)  # 修改最后一个参数
                # cv2.namedWindow("projector", cv2.WND_PROP_FULLSCREEN)
                # cv2.setWindowProperty("projector", cv2.WND_PROP_FULLSCREEN, cv2.WINDOW_FULLSCREEN)
                cv2.imshow("projector", img)
                #cv2.waitKey(1)
                pos = []

            elif coor == 2:
                if pick_start:
                    for i in range(int(len(pos) / 2)):
                        img = draw_x(img,pos[2*i],pos[2*i + 1], 40)  # 修改最后一个参数
                    pick_start = False
                    # cv2.namedWindow("projector", cv2.WND_PROP_FULLSCREEN)
                    # cv2.setWindowProperty("projector", cv2.WND_PROP_FULLSCREEN, cv2.WINDOW_FULLSCREEN)
                    cv2.imshow("projector", img)
                    time.sleep(5)
                pos = []

            elif coor == 0:
                pick_start = False
                img = np.zeros((900, 1600, 3), np.uint8)
                # cv2.namedWindow("projector", cv2.WND_PROP_FULLSCREEN)
                # cv2.setWindowProperty("projector", cv2.WND_PROP_FULLSCREEN, cv2.WINDOW_FULLSCREEN)
                cv2.imshow("projector", img)
                #cv2.waitKey(1)
                pos = []

            elif coor == -8:
                pick_start = False
                img = np.zeros((900, 1600, 3), np.uint8)
                i = 0
                if len(pos) == 2:
                    cv2.circle(img, (pos[0], pos[1]), 150, (255, 255, 255), 5)
                # cv2.namedWindow("projector", cv2.WND_PROP_FULLSCREEN)
                # cv2.setWindowProperty("projector", cv2.WND_PROP_FULLSCREEN, cv2.WINDOW_FULLSCREEN)
                cv2.imshow("projector", img)
                time.sleep(1)
                pos = []

            else:
                cv2.imshow("projector", img)


        else:
            if not pick_start:
                img = np.zeros((900, 1600, 3), np.uint8)
                cv2.imshow("projector", img)
                # cv2.namedWindow("projector", cv2.WND_PROP_FULLSCREEN)
                # cv2.setWindowProperty("projector", cv2.WND_PROP_FULLSCREEN, cv2.WINDOW_FULLSCREEN)
                # cv2.imshow("projector", img)
                #cv2.waitKey(1)
                pos = []

            else:
                cv2.imshow("projector", img)

    else:
        pos.append(coor)
        cv2.imshow("projector", img)

cv2.destroyAllWindows()
server.close()





