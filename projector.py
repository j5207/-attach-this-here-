#!/usr/bin/env python
from __future__ import print_function
import numpy as np
import time
import cv2
import rospy
from std_msgs.msg import Int32MultiArray



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

class projector():
    def __init__(self):
        rospy.init_node("projector")
        self.sub = rospy.Subscriber('/netsend', Int32MultiArray, self.projector_cb)
        #intro()
        self.pick_start = False
        self.place_finish = False
        self.img = np.zeros((900, 1600, 3), np.uint8) * 255
        cv2.namedWindow("projector1", cv2.WND_PROP_FULLSCREEN)
        cv2.setWindowProperty("projector1", cv2.WND_PROP_FULLSCREEN, cv2.WINDOW_FULLSCREEN)
        self.pos = None
        self.coor = None

    def projector_cb(self, msg):
        a = list(msg.data)
        pos = a[:-1]
        self.coor = a[-1]
        self.pos = coor_tran(pos)

if __name__ == '__main__':
    proj = projector()
    while True:
        if proj.pos:

            if proj.coor == -1 and not proj.pick_start:
                proj.img = np.zeros((900, 1600, 3), np.uint8)
                for i in range(int(len(proj.pos) / 2)):
                    cv2.circle(proj.img, (proj.pos[2*i] + 20, proj.pos[2*i + 1]), 40, (255, 255, 255), -1)
                cv2.imshow('projector1' , proj.img)
                cv2.waitKey(1)

            elif proj.coor == 1:
                rospy.loginfo("rect pos:{}".format(proj.pos))
                proj.pick_start = True
                proj.img = np.zeros((900, 1600, 3), np.uint8)
                for i in range(int(len(proj.pos) / 2)):
                    cv2.rectangle(proj.img, (proj.pos[2*i]-30, proj.pos[2*i + 1]-30), (proj.pos[2*i]+30, proj.pos[2*i+1]+ 30), (255, 255, 255), -1)
                cv2.imshow('projector1' , proj.img)
                cv2.waitKey(1)

            elif proj.coor == 2 and proj.pick_start:
                rospy.loginfo("cross pos:{}".format(proj.pos))
                for i in range(int(len(proj.pos) / 2)):
                    proj.img = draw_x(proj.img,proj.pos[2*i],proj.pos[2*i + 1], 40) 
                proj.pick_start = False
                proj.place_finish = True
                rospy.loginfo("placement")
                cv2.imshow('projector1' , proj.img)
                cv2.waitKey(1)
                rospy.sleep(3)

            elif proj.coor == 0:
                proj.pick_start = False
                proj.img = np.zeros((900, 1600, 3), np.uint8)
                cv2.imshow('projector1' , proj.img)
                cv2.waitKey(1)

            elif proj.coor == -8:
                rospy.loginfo("circle pos:{}".format(proj.pos))
                proj.pick_start = True
                proj.img = np.zeros((900, 1600, 3), np.uint8)
                if len(proj.pos) == 2:
                    cv2.circle(proj.img, (proj.pos[0], proj.pos[1]), 100, (255, 255, 255), 5)
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