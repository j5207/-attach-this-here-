from __future__ import print_function
import cv2
import numpy as np
import heapq
from utils import cache
from constant import Hand_low, Hand_high
from utils import cache
import math

Finger_distanct = 20

class hand_tracking():
    def __init__(self, image, memory1, memory2):
        self.memory1 = memory1
        self.memory2 = memory2
        self.flag = False

        frame = image.copy()
        self.radius_thresh = 0.05

        blur = cv2.blur(frame,(3,3))
        hsv = cv2.cvtColor(blur,cv2.COLOR_BGR2HSV)
    
        mask = cv2.inRange(hsv, Hand_low, Hand_high)
    
        _, contours, _ = cv2.findContours(mask,cv2.RETR_TREE,cv2.CHAIN_APPROX_SIMPLE)  
        self.num_hands = 0
        self.hand_cnt = [] 
        self.finger = []
        # self.rl_point = None
        self.center = []
        # self.angle = None
        # self.twoangle = None
        max_area = 1000
        for i in range(len(contours)):
            cnt=contours[i]
            area = cv2.contourArea(cnt)
            if(area>max_area):
                cnts = contours[i]
                self.num_hands += 1
                epsilon = 0.001*cv2.arcLength(cnt,True)
                approx = cv2.approxPolyDP(cnt,epsilon,True)
                hull = cv2.convexHull(cnts)
                frame,hand_center,hand_radius = self.mark_hand_center(frame, cnts)
                
                frame,finger=self.mark_fingers(frame,hull,hand_center,hand_radius)
                cv2.drawContours(frame,[approx],-1,(0, 0, 255),1)
                self.hand_cnt.append([approx])

                
        cv2.imshow('hand_tracking',frame) 
    def get_result(self):
        #self.filter()
        # return (self.only_point, self.angle), (self.rl_point, self.twoangle), self.center
        print(self.finger, self.center)
        return self.finger, self.center
    
    # def filter(self):
    #     if self.memory1.full:
    #         if 0 in self.memory1.list:
    #             self.only_point = None
    #         self.memory1.clear()
    #     else:
    #         self.only_point = None

    #     if self.memory2.full:
    #         if 0 in self.memory2.list:
    #             self.rl_point = None
    #         self.memory2.clear()
    #     else:
    #         self.rl_point = None
    #     # self.memory.append()

    def mark_hand_center(self, frame_in,cont):    
        max_d=0
        pt=(0,0)
        x,y,w,h = cv2.boundingRect(cont)
        for ind_y in xrange(int(y),int(y+h)): #around 0.25 to 0.6 region of height (Faster calculation with ok results)
            for ind_x in xrange(int(x),int(x+w)): #around 0.3 to 0.6 region of width (Faster calculation with ok results)
                dist= cv2.pointPolygonTest(cont,(ind_x,ind_y),True)
                if(dist>max_d):
                    max_d=dist
                    pt=(ind_x,ind_y)
        cv2.circle(frame_in,pt,int(max_d),(255,0,0),2)
        return frame_in,pt,max_d

    def mark_fingers(self, frame_in,hull,pt,radius):
      
        finger=[(hull[0][0][0],hull[0][0][1])]
        j=0

        cx = pt[0]
        cy = pt[1]
        self.center.append((cx, cy))
        for i in range(len(hull)):
            dist = np.sqrt((hull[-i][0][0] - hull[-i+1][0][0])**2 + (hull[-i][0][1] - hull[-i+1][0][1])**2)
            if dist>Finger_distanct:
                if(j==0):
                    finger=[(hull[-i][0][0],hull[-i][0][1])]
                else:
                    finger.append((hull[-i][0][0],hull[-i][0][1]))
                j=j+1

        #finger = filter(lambda x: x[0] < cx, finger)
        finger = filter(lambda x: np.sqrt((x[0]- cx)**2 + (x[1] - cy)**2) > 1.7 * radius, finger)
        dis_center_ls = []        
        for i in range(len(finger)):
            dist = np.sqrt((finger[i][0]- cx)**2 + (finger[i][1] - cy)**2)
            dis_center_ls.append(dist)
        if len(dis_center_ls) >= 2:
            largest_two = heapq.nlargest(2, dis_center_ls)
            
            if largest_two[0] > largest_two[1] * 1.3 and largest_two[0]>70 and radius>26 and largest_two[0] - largest_two[1] > 30 and len(dis_center_ls)<3:
                #print largest_two[0] , largest_two[1]
                cv2.putText(frame_in,"pointing",(int(0.38*frame_in.shape[1]),int(0.12*frame_in.shape[0])),cv2.FONT_HERSHEY_DUPLEX,1,(0,255,255),1,8)
                self.finger.append(finger[dis_center_ls.index(largest_two[0])])

        # elif len(dis_center_ls) == 2:
        #     cv2.putText(frame_in,"two finger pointing",(int(0.38*frame_in.shape[1]),int(0.12*frame_in.shape[0])),cv2.FONT_HERSHEY_DUPLEX,1,(0,255,255),1,8)
        #     # self.right = finger[0]
        #     # self.left = finger[1]
        #     self.rl_point = (finger[0], finger[1])

        elif len(dis_center_ls) == 1:
            if dis_center_ls[0] > 70:
                #print "only, {}".format(dis_center_ls[0])
                cv2.putText(frame_in,"pointing",(int(0.38*frame_in.shape[1]),int(0.12*frame_in.shape[0])),cv2.FONT_HERSHEY_DUPLEX,1,(0,255,255),1,8)
                self.finger.append(finger[0])
        '''
        add something unrelevant
        '''
        


        
        # if self.only_point is not None:
        #     self.memory1.append(1)
        #     px, py = self.only_point
        #     cx, cy = self.center
        #     self.angle = math.atan2(cy-py, cx-px)
        #     self.angle = np.rad2deg(self.angle)
        #     # print(self.angle)
        # else:
        #     self.memory1.append(0)

        # if self.rl_point:
        #     self.memory2.append(1)
        #     (rx, ry), (lx, ly) = self.rl_point
        #     cx, cy = self.center
        #     r_angle = math.atan2(cy-ry, cx-rx)
        #     r_angle = np.rad2deg(r_angle)
        #     l_angle = math.atan2(cy-ly, cx-lx)
        #     l_angle = np.rad2deg(l_angle)
        #     self.twoangle = (r_angle, l_angle)
        # else:
        #     self.memory2.append(0)
       

        for k in range(len(finger)):
            cv2.circle(frame_in,finger[k],10,255,2)
            cv2.line(frame_in,finger[k],(cx,cy),255,2)
        return frame_in,finger
    
def warp(img):
    #pts1 = np.float32([[115,124],[520,112],[2,476],[640,480]])
    pts1 = np.float32([[268,76],[500,58],[272,252],[523,237]])
    pts2 = np.float32([[0,0],[640,0],[0,480],[640,480]])
    M = cv2.getPerspectiveTransform(pts1,pts2)
    dst = cv2.warpPerspective(img,M,(640,480))
    return dst
    
        



if __name__ == '__main__':
    cap = cv2.VideoCapture(0)
    while True:
        OK, origin = cap.read()
        ob = hand_tracking(warp(origin), cache(10), cache(10))
        ob.get_result()
        # if ob.angle is not None:
        #     print(ob.angle)
        k = cv2.waitKey(1) & 0xFF # large wait time to remove freezing
        if k == 113 or k == 27:
            break
    cap.release()
    cv2.destroyAllWindows()
