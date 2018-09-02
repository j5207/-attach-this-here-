#!/usr/bin/env python
from __future__ import print_function
import numpy as np
import time
import cv2
import rospy
from std_msgs.msg import Int32MultiArray


def draw_rec(img):
    cv2.line(img, (49, 124), (49,325), (255, 255, 255), 5)
    cv2.line(img, (49, 325), (473,325), (255, 255, 255), 5)
    cv2.line(img, (473, 325), (473,124), (255, 255, 255), 5)
    cv2.line(img, (473, 124), (49,124), (255, 255, 255), 5)
    return img

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
    cv2.arrowedLine(img, (int(0.6*x1 + 0.4*x2), int(0.6*y1 + 0.4*y2)), (int(0.4*x1 + 0.6*x2), int(0.4*y1+0.6*y2)),(255, 255, 255), line)
    return img

def draw_tran(img, x, y, line):
    points = np.array([(x + int(line*1.732/2), y - int(line/2)), (x - int(line*1.732/2), y - int(line/2)), (x, y + line)])
    cv2.drawContours(img, [points], 0, (255,255,255), -1)
    return img

def white():
    img = np.ones((900, 1600, 3), np.uint8) * 255
    cv2.namedWindow("projector", cv2.WND_PROP_FULLSCREEN)
    cv2.setWindowProperty("projector", cv2.WND_PROP_FULLSCREEN, cv2.WINDOW_FULLSCREEN)
    cv2.imshow("projector", img)
    cv2.waitKey(1)

def black():
    img = np.zeros((900, 1600, 3), np.uint8) * 255
    cv2.namedWindow("projector", cv2.WND_PROP_FULLSCREEN)
    cv2.setWindowProperty("projector", cv2.WND_PROP_FULLSCREEN, cv2.WINDOW_FULLSCREEN)
    cv2.imshow("projector", img)
    cv2.waitKey(20)

class projector():
    def __init__(self):
        rospy.init_node("projector")
        self.sub = rospy.Subscriber('/netsend', Int32MultiArray, self.projector_cb)
        #intro()
        self.flag = 1
        self.pick_start = False
        self.place_finish = False
        self.img = np.zeros((900, 1600, 3), np.uint8) * 255
        cv2.namedWindow("projector1", cv2.WND_PROP_FULLSCREEN)
        cv2.setWindowProperty("projector1", cv2.WND_PROP_FULLSCREEN, cv2.WINDOW_FULLSCREEN)
        self.pos = None
        self.coor = None
        self.multar = None
        self.evenx = 0
        self.eveny = 0
        self.cancelling = False


    def projector_cb(self, msg):
        a = list(msg.data)
        pos = a[:-1]
        self.coor = a[-1]
        self.pos = coor_tran(pos)
        print(self.pos)
        print(self.coor)

if __name__ == '__main__':
    proj = projector()
    temp = 0
    while True:
        if proj.pos:
            temp_pos = proj.pos
            temp_coor = proj.coor
            if temp_coor == -1 and not proj.pick_start:
                
                proj.img = np.zeros((900, 1600, 3), np.uint8)
                try:
                    for i in range(int(len(temp_pos) / 2)):
                        cv2.circle(proj.img, (temp_pos[2*i], temp_pos[2*i + 1]), 40, (255, 255, 255), -1)
                except IndexError:
                    print(i)
                    print(temp_pos)
                cv2.imshow('projector1' , proj.img)
                cv2.waitKey(1)

            elif temp_coor == 1:
                rospy.loginfo("rect pos:{}".format(temp_pos))
                proj.cancelling = False
                proj.pick_start = True
                if not proj.pick_start:
                    proj.img = np.zeros((900, 1600, 3), np.uint8)
                if len(temp_pos) > 2:
                    proj.multar = temp_pos

                    if proj.evenx == 0 or len(temp_pos) > temp:
                        proj.evenx = 0
                        proj.eveny = 0
                        for i in range(int(len(temp_pos) / 2)):
                            proj.evenx = proj.evenx + temp_pos[2*i]
                            proj.eveny = proj.eveny + temp_pos[2*i + 1]
                        proj.evenx = int(proj.evenx / (len(temp_pos) / 2))
                        proj.eveny = int(proj.eveny / (len(temp_pos) / 2))
                        temp = len(temp_pos)
                    
                for i in range(int(len(temp_pos) / 2)):
                    print("coor:")
                    print(temp_coor)
                    print("pos:")
                    print(temp_pos)
                    if temp_coor != 2:
                        cv2.rectangle(proj.img, (temp_pos[2*i]-40, temp_pos[2*i + 1]-40), (temp_pos[2*i]+40, temp_pos[2*i+1]+ 40), (255, 255, 255), -1)
                        cv2.imshow('projector1' , proj.img)
                        cv2.waitKey(1)

            elif temp_coor == 2 and proj.pick_start:
                rospy.loginfo("cross pos:{}".format(temp_pos))
                for i in range(int(len(temp_pos) / 2)):
                    proj.img = draw_x(proj.img,temp_pos[2*i],temp_pos[2*i + 1], 40) 
                if proj.multar is not None:
                    
                    for i in range(int(len(proj.multar) / 2)):
                        proj.multar[2*i]
                        proj.img = draw_tran(proj.img,temp_pos[0] + proj.multar[2*i] - proj.evenx, temp_pos[1] + proj.multar[2*i + 1] - proj.eveny, 40) 
                    proj.multar = None
                    proj.evenx = 0
                    proj.eveny = 0
                proj.pick_start = False
                proj.place_finish = True
                rospy.loginfo("placement")
                cv2.imshow('projector1' , proj.img)
                cv2.waitKey(0)
                if proj.flag == 1:
                    img = np.zeros((900, 1600, 3), np.uint8) * 255
                    cv2.circle(img, (int(1600*500/640),int(250*900/480)), 40, (255, 255, 255), -1)
                    img = draw_x(img,int(325*1600/640),int(350*900/480 - 20), 40)
                    img = draw_arrow(img, int(1600*500/640), int(250*900/480), int(1600*325/640),int(350*900/480 - 20),5)
                    cv2.imshow("projector1", img)
                    cv2.waitKey(10000)
                    proj.img = np.zeros((900, 1600, 3), np.uint8)
                    cv2.imshow('projector1' , proj.img)
                    cv2.waitKey(1)
                    proj.flag = 2
                    

                elif proj.flag == 2:
                    img = np.zeros((900, 1600, 3), np.uint8) * 255
                    cv2.circle(img, (int(1600*200/640),int(250*900/480)), 40, (255, 255, 255), -1)
                    cv2.circle(img, (int(1600*200/640+150),int(250*900/480)), 40, (255, 255, 255), -1)
                    cv2.circle(img, (int(1600*200/640+75),int(250*900/480+80)), 40, (255, 255, 255), -1)
                    img = draw_x(img,int(325*1600/640),int(350*900/480 - 20), 40)
                    img = draw_arrow(img, int(1600*200/640+75), int(250*900/480), int(1600*325/640),int(350*900/480 - 20),5)
                    cv2.imshow("projector1", img)
                    cv2.waitKey(10000)
                    proj.img = np.zeros((900, 1600, 3), np.uint8)
                    cv2.imshow('projector1' , proj.img)
                    cv2.waitKey(1)
                    proj.flag = 3

                elif proj.flag == 3:
                    img = np.zeros((900, 1600, 3), np.uint8) * 255
                    cv2.circle(img, (int(1600*150/640 +50),int(150*900/480)), 120, (255, 255, 255), 5)
                    img = draw_x(img,int(325*1600/640),int(350*900/480 - 20), 40)
                    img = draw_arrow(img, int(1600*150/640 + 50), int(150*900/480 + 20), int(1600*325/640),int(350*900/480 - 20),5)
                    cv2.imshow("projector1", img)
                    cv2.waitKey(10000)
                    proj.img = np.zeros((900, 1600, 3), np.uint8)
                    cv2.imshow('projector1' , proj.img)
                    cv2.waitKey(1)
                    proj.flag = 4

                elif proj.flag == 4:
                    img = np.zeros((900, 1600, 3), np.uint8) * 255
                    cv2.circle(img, (int(1600*400/640 + 40),int(300*900/480)), 120, (255, 255, 255), 5)
                    img = draw_x(img,int(325*1600/640),int(350*900/480 - 20), 40)
                    img = draw_arrow(img, int(1600*400/640 + 40), int(300*900/480), int(1600*325/640),int(350*900/480 - 20),5)
                    cv2.imshow("projector1", img)
                    cv2.waitKey(10000)
                    proj.img = np.zeros((900, 1600, 3), np.uint8)
                    cv2.imshow('projector1' , proj.img)
                    cv2.waitKey(1)
                    proj.flag = 5

                elif proj.flag == 5:
                    img = np.zeros((900, 1600, 3), np.uint8) * 255
                    cv2.circle(img, (int(1600*275/640),int(190*900/480)), 40, (255, 255, 255), -1)
                    cv2.circle(img, (int(1600*275/640+150),int(190*900/480)), 40, (255, 255, 255), -1)
                    img = draw_x(img,int(325*1600/640),int(350*900/480 - 20), 40)
                    img = draw_arrow(img, int(1600*275/640+75), int(190*900/480), int(1600*325/640),int(350*900/480 - 20),5)
                    cv2.imshow("projector1", img)
                    cv2.waitKey(10000)
                    proj.img = np.zeros((900, 1600, 3), np.uint8)
                    cv2.imshow('projector1' , proj.img)
                    cv2.waitKey(1)
                    proj.flag = 0

            elif temp_coor == 0:
                proj.pick_start = False
                proj.img = np.ones((900, 1600, 3), np.uint8) * 255
                cv2.imshow('projector1' , proj.img)
                cv2.waitKey(1)

            elif temp_coor == -8:
                rospy.loginfo("circle pos:{}".format(temp_pos))
                proj.pick_start = True
                proj.img = np.zeros((900, 1600, 3), np.uint8)
                if len(temp_pos) == 2:
                    cv2.ellipse(proj.img, (temp_pos[0], temp_pos[1]), (1600*100/640,100*900/480), 0,0,360,(255, 255, 255), 5)
                cv2.imshow('projector1' , proj.img)
                cv2.waitKey(1)
            
            else:
                cv2.imshow('projector1' , proj.img)
                cv2.waitKey(1)
            

        else:
            if not proj.pick_start:
                proj.img = np.zeros((900, 1600, 3), np.uint8)
                cv2.imshow('projector1' , proj.img)
                cv2.waitKey(1)

        k = 0xFF # large wait time to remove freezing
        if k == 113 or k == 27:
            break
    cv2.destroyAllWindows()
    rospy.spin()