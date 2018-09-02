
from __future__ import print_function
import numpy as np
import time
import cv2
import rospy
from std_msgs.msg import Int32MultiArray



def draw_tran(img, x, y, line):
    points = [[x + int(line*1.732/2), y - int(line/2)], [x - int(line*1.732/2), y - int(line/2)], [x, y + line]]
    cv2.fillPoly(img, np.int32([points]),(255,255,255), 4, 2)



img = np.ones((900, 1600, 3), np.uint8) * 255
    
line = 40
img = draw_tran(proj.img,proj.pos[0] + proj.multar[2*i] - proj.evenx, proj.pos[1] + proj.multar[2*i + 1] - proj.eveny, 40) 

points = [[x + int(line*1.732/2), y - int(line/2)], [x - int(line*1.732/2), y - int(line/2)], [x, y + line]]
cv2.fillPoly(img, np.int32([points]),(255,255,255), 4, 2)


cv2.namedWindow("projector", cv2.WND_PROP_FULLSCREEN)
cv2.setWindowProperty("projector", cv2.WND_PROP_FULLSCREEN, cv2.WINDOW_FULLSCREEN)
cv2.imshow("projector", img)
cv2.waitKey(0)