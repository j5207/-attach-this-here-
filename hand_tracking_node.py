#!/usr/bin/env python
from __future__ import print_function
import cv2
import numpy as np
from constant import *
from joblib import load
from utils import side_finder, test_insdie, cache
from hand_tracking import hand_tracking
from shapely.geometry import Polygon
from math import sqrt
from copy import deepcopy
import rospy
from std_msgs.msg import Int32MultiArray, Int32

distant = lambda (x1, y1), (x2, y2) : sqrt((x1 - x2)**2 + (y1 - y2)**2)
voice_flag = 0

def warp_img(img):
    #pts1 = np.float32([[115,124],[520,112],[2,476],[640,480]])
    pts1 = np.float32([[268,76],[500,58],[272,252],[523,237]])
    pts2 = np.float32([[0,0],[640,0],[0,480],[640,480]])
    M = cv2.getPerspectiveTransform(pts1,pts2)
    dst = cv2.warpPerspective(img,M,(640,480))
    return dst

def camrectify(frame):
        mtx = np.array([
            [509.428319, 0, 316.944024],
            [0.000000, 508.141786, 251.243128],
            [0.000000, 0.000000, 1.000000]
        ])
        dist = np.array([
            0.052897, -0.155430, 0.005959, 0.002077, 0.000000
        ])
        return cv2.undistort(frame, mtx, dist)

def get_crit(mask):
    (_,contours, hierarchy)=cv2.findContours(mask,cv2.RETR_TREE,cv2.CHAIN_APPROX_SIMPLE)
    max_area = 0
    crit = None
    for i , contour in enumerate(contours):
        area = cv2.contourArea(contour)
        if area > max_area and hierarchy[0, i, 3] == -1:
            max_area = area
            crit = area
    return crit

def get_objectmask(img):
    hsv = cv2.cvtColor(img,cv2.COLOR_BGR2HSV)
    green_mask = cv2.inRange(hsv, Green_low, Green_high)
    hand_mask = cv2.inRange(hsv, Hand_low, Hand_high)
    hand_mask = cv2.dilate(hand_mask, kernel = np.ones((11,11),np.uint8))
    skin_mask = cv2.inRange(hsv, Skin_low, Skin_high)
    skin_mask = cv2.dilate(skin_mask, kernel = np.ones((11,11),np.uint8))
    thresh = 255 - green_mask
    thresh = cv2.subtract(thresh, hand_mask)
    thresh = cv2.subtract(thresh, skin_mask)
    thresh[477:, 50:610] = 0
    return thresh

def get_k_dis((x1, y1), (x2, y2), (x, y)):
        coord = ((x, y), (x1, y1), (x2, y2))
        return Polygon(coord).area / distant((x1, y1), (x2, y2))

class temp_tracking():
    def __init__(self):
        self.cap = cv2.VideoCapture(0)

    def update(self):
        '''
        gesture flag for distinguish different scenario
        '''
        self.surfacels = []
        self.boxls = []
        OK, origin = self.cap.read()
        x = None
        if OK:
            rect = camrectify(origin)
            warp = warp_img(rect)
            thresh = get_objectmask(warp)
            cv2.imshow('thresh', thresh)
            self.image = warp.copy()
            draw_img1 = warp.copy()
            self.get_bound(draw_img1, thresh, visualization=True)
            cx, cy = None, None
            lx, rx = None, None
            result = hand_tracking(warp_img(origin), cache(10), cache(10)).get_result()
            num_hand_view = len(result)
            '''
            one hand in the view
            '''
            if num_hand_view == 1:
                center = result[0][0]
                tips = result[0][1]
                num_tips = len(tips)
            # '''
            # one hand and one finger, flag == 1
            # '''
                if num_tips == 1 and len(self.boxls) > 0:
                    point = tips[0]
                    length_ls = []
                    for x, y, w, h in self.boxls:
                        length_ls.append((get_k_dis((point[0], point[1]), (center[0], center[1]), (x+w/2, y+h/2)), (x+w/2, y+h/2)))
                    length_ls = filter(lambda x: (point[1] - x[1][1]) * (point[1] - center[1]) <= 0, length_ls)
                    if len(length_ls) > 0:
                        x,y = min(length_ls, key=lambda x: x[0])[1]
                        ind = test_insdie((x, y), self.boxls)
                        x, y, w, h = self.boxls[ind]
                        cx, cy = self.surfacels[ind]
                        cv2.rectangle(draw_img1,(x,y),(x+w,y+h),(0,0,255),2)
                        cv2.circle(draw_img1, (cx, cy), 5, (0, 0, 255), -1)
                        cv2.putText(draw_img1,"pointed",(x,y),cv2.FONT_HERSHEY_SIMPLEX, 1.0,(0,0,255))
                        '''
                        flag is 1
                        '''
                        return [[point[0],point[1]],[cx,cy], 1]
            #  '''
            # one hand and two finger, flag == 2
            # '''
                if num_tips == 2 and len(self.boxls) > 0:
                    boxls = deepcopy(self.boxls)
                    length_lsr = []
                    length_lsl = []
                    rpoint, lpoint = tips
                    for x, y, w, h in self.boxls:
                        length_lsr.append((get_k_dis((rpoint[0], rpoint[1]), (center[0], center[1]), (x+w/2, y+h/2)), (x+w/2, y+h/2)))
                    rx,ry = min(length_lsr, key=lambda x: x[0])[1]
                    rind = test_insdie((rx, ry), self.boxls)
                    x, y, w, h = self.boxls[rind]
                    del boxls[rind]
                    cv2.rectangle(draw_img1,(x,y),(x+w,y+h),(0,0,255),2)
                    cv2.putText(draw_img1,"pointed_right",(x,y),cv2.FONT_HERSHEY_SIMPLEX, 1.0,(0,0,255))
                    if len(boxls) > 0:
                        for x, y, w, h in boxls:
                            length_lsl.append((get_k_dis((lpoint[0], lpoint[1]), (center[0], center[1]), (x+w/2, y+h/2)), (x+w/2, y+h/2)))
                        lx,ly = min(length_lsl, key=lambda x: x[0])[1]
                        lind = test_insdie((lx, ly), boxls)
                        x, y, w, h = boxls[lind]
                        cv2.rectangle(draw_img1,(x,y),(x+w,y+h),(0,0,255),2)
                        cv2.putText(draw_img1,"pointed_left",(x,y),cv2.FONT_HERSHEY_SIMPLEX, 1.0,(0,0,255))
                        return [[tips[0][0], tips[0][1]], [tips[1][0], tips[1][1]], [rx, ry], [lx, ly], 2]
            '''
            two hand in the view
            '''
            if num_hand_view == 2:
                lcenter = result[0][0]
                ltips = result[0][1]
                lnum_tips = len(ltips)

                rcenter = result[1][0]
                rtips = result[1][1]
                rnum_tips = len(rtips)
            # '''
            # one hand is one finger tip and another one not pointing, ONLY PICK
            # '''
                if set([lnum_tips, rnum_tips]) == set([0, 1]) and len(self.boxls) > 0:
                    sub_result = filter(lambda x: len(x[1]) == 1, result)
                    center = sub_result[0]
                    tips = sub_result[1]
                    point = tips[0]
                    length_ls = []
                    for x, y, w, h in self.boxls:
                        length_ls.append((get_k_dis((point[0], point[1]), (center[0], center[1]), (x+w/2, y+h/2)), (x+w/2, y+h/2)))
                    length_ls = filter(lambda x: (point[1] - x[1][1]) * (point[1] - center[1]) <= 0, length_ls)
                    if len(length_ls) > 0:
                        x,y = min(length_ls, key=lambda x: x[0])[1]
                        ind = test_insdie((x, y), self.boxls)
                        x, y, w, h = self.boxls[ind]
                        cx, cy = self.surfacels[ind]
                        cv2.rectangle(draw_img1,(x,y),(x+w,y+h),(0,0,255),2)
                        cv2.circle(draw_img1, (cx, cy), 5, (0, 0, 255), -1)
                        cv2.putText(draw_img1,"pointed",(x,y),cv2.FONT_HERSHEY_SIMPLEX, 1.0,(0,0,255))
                        '''
                        flag is 1
                        '''
                        return [[point[0],point[1]],[cx,cy], 1]
                # '''
                # two hand is both one finger pointing, ONLY PLACE
                # '''
                if set([lnum_tips, rnum_tips]) == set([1,1]) and len(self.boxls) > 0:
                    boxls = deepcopy(self.boxls)
                    length_lsr = []
                    length_lsl = []
                    rpoint, lpoint = rtips[0], ltips[0]
                    for x, y, w, h in self.boxls:
                        length_lsr.append((get_k_dis((rpoint[0], rpoint[1]), (rcenter[0], rcenter[1]), (x+w/2, y+h/2)), (x+w/2, y+h/2)))
                    rx,ry = min(length_lsr, key=lambda x: x[0])[1]
                    rind = test_insdie((rx, ry), self.boxls)
                    x, y, w, h = self.boxls[rind]
                    del boxls[rind]
                    cv2.rectangle(draw_img1,(x,y),(x+w,y+h),(0,0,255),2)
                    cv2.putText(draw_img1,"pointed_right",(x,y),cv2.FONT_HERSHEY_SIMPLEX, 1.0,(0,0,255))
                    if len(boxls) > 0:
                        for x, y, w, h in boxls:
                            length_lsl.append((get_k_dis((lpoint[0], lpoint[1]), (lcenter[0], lcenter[1]), (x+w/2, y+h/2)), (x+w/2, y+h/2)))
                        lx,ly = min(length_lsl, key=lambda x: x[0])[1]
                        lind = test_insdie((lx, ly), boxls)
                        x, y, w, h = boxls[lind]
                        cv2.rectangle(draw_img1,(x,y),(x+w,y+h),(0,0,255),2)
                        cv2.putText(draw_img1,"pointed_left",(x,y),cv2.FONT_HERSHEY_SIMPLEX, 1.0,(0,0,255))
                        '''
                        flag is 3
                        '''
                        return [[rtips[0][0], rtips[0][1]], [ltips[0][0], ltips[0][1]], [rx, ry], [lx, ly], 3]

                    
                    

                
        cv2.imshow('draw', draw_img1)
        # if cx and point:
        #     return [[point[0],point[1]],[cx,cy]]
        # if point2:
        #     if lx and rx:
        #         return [[point2[0][0], point2[0][1]], [point2[1][0], point2[1][1]], [rx, ry], [lx, ly]]
        # if x and point:
        #     return [[point[0],point[1]],[x+w/2,y+h/2]]

    def get_bound(self, img, object_mask, visualization=True):
        (_,object_contours, object_hierarchy)=cv2.findContours(object_mask,cv2.RETR_TREE,cv2.CHAIN_APPROX_SIMPLE)
        if len(object_contours) > 0:
            for i , contour in enumerate(object_contours):
                area = cv2.contourArea(contour)
                if area>200 and area < 2000 and object_hierarchy[0, i, 3] == -1:					
                    M = cv2.moments(contour)
                    cx = int(M['m10']/M['m00'])
                    cy = int(M['m01']/M['m00'])
                    x,y,w,h = cv2.boundingRect(contour)
                    self.boxls.append((x, y, w, h))
        if len(self.boxls) > 0:
            boxls_arr = np.array(self.boxls)
            self.boxls = boxls_arr[boxls_arr[:, 0].argsort()].tolist()
        for x, y, w, h in self.boxls:
            sub = self.image[y:y+h, x:x+w, :]
            hsv = cv2.cvtColor(sub,cv2.COLOR_BGR2HSV)
            top_mask = cv2.inRange(hsv, Top_low, Top_high)
            (_,top_contours, object_hierarchy)=cv2.findContours(top_mask,cv2.RETR_TREE,cv2.CHAIN_APPROX_SIMPLE)
            max_area = 0
            for i , contour in enumerate(top_contours):
                area = cv2.contourArea(contour)
                if area>max_area and object_hierarchy[0, i, 3] == -1:					
                    M = cv2.moments(contour)
                    cx = int(M['m10']/M['m00'])
                    cy = int(M['m01']/M['m00'])
                    max_area = area
            self.surfacels.append((cx+x, cy+y))

    def __del__(self):
        self.cap.release()

def callback(msg):
    print(msg.data)
    global voice_flag
    voice_flag = msg.data


if __name__ == '__main__':
    temp = temp_tracking()
    rospy.init_node('hand_tracking_node')
    pub = rospy.Publisher('/target_position', Int32MultiArray, queue_size=20)
    sub = rospy.Subscriber('/voice_command', Int32, callback)
    ready_for_place = False
    pos_N_cmd = []
    while True:
        pos = temp.update()
        if pos:
            if voice_flag == 1 and not ready_for_place:
                if len(pos) == 2:
                    pos_N_cmd.append(int(pos[1][0]))
                    pos_N_cmd.append(int(pos[1][1]))
                elif len(pos) == 4:
                    pos_N_cmd.append(int(pos[2][0]))
                    pos_N_cmd.append(int(pos[2][1]))
                    pos_N_cmd.append(int(pos[3][0]))
                    pos_N_cmd.append(int(pos[3][1]))
                print(pos_N_cmd)
                ready_for_place = True
            elif voice_flag == 2 and ready_for_place:
                pos_N_cmd.append(int(pos[0][0]))
                pos_N_cmd.append(int(pos[0][1]))
                print(pos_N_cmd)
                pub.publish(Int32MultiArray(data=pos_N_cmd))
                ready_for_place = False
                pos_N_cmd = []
            elif voice_flag == -1:
                break
        k = cv2.waitKey(1) & 0xFF # large wait time to remove freezing
        if k == 113 or k == 27:
            break
    cv2.destroyAllWindows()

# #!/usr/bin/env python
# from __future__ import print_function
# import cv2
# import numpy as np
# from constant import *
# from joblib import load
# from utils import side_finder, test_insdie, cache
# from hand_tracking import hand_tracking
# from shapely.geometry import Polygon
# from math import sqrt
# from copy import deepcopy
# import rospy
# from std_msgs.msg import Int32MultiArray, Int32

# distant = lambda (x1, y1), (x2, y2) : sqrt((x1 - x2)**2 + (y1 - y2)**2)
# voice_flag = 0

# def warp_img(img):
#     #pts1 = np.float32([[115,124],[520,112],[2,476],[640,480]])
#     pts1 = np.float32([[268,76],[500,58],[272,252],[523,237]])
#     pts2 = np.float32([[0,0],[640,0],[0,480],[640,480]])
#     M = cv2.getPerspectiveTransform(pts1,pts2)
#     dst = cv2.warpPerspective(img,M,(640,480))
#     return dst

# def camrectify(frame):
#         mtx = np.array([
#             [509.428319, 0, 316.944024],
#             [0.000000, 508.141786, 251.243128],
#             [0.000000, 0.000000, 1.000000]
#         ])
#         dist = np.array([
#             0.052897, -0.155430, 0.005959, 0.002077, 0.000000
#         ])
#         return cv2.undistort(frame, mtx, dist)

# def get_crit(mask):
#     (_,contours, hierarchy)=cv2.findContours(mask,cv2.RETR_TREE,cv2.CHAIN_APPROX_SIMPLE)
#     max_area = 0
#     crit = None
#     for i , contour in enumerate(contours):
#         area = cv2.contourArea(contour)
#         if area > max_area and hierarchy[0, i, 3] == -1:
#             max_area = area
#             crit = area
#     return crit

# def get_objectmask(img):
#     hsv = cv2.cvtColor(img,cv2.COLOR_BGR2HSV)
#     green_mask = cv2.inRange(hsv, Green_low, Green_high)
#     hand_mask = cv2.inRange(hsv, Hand_low, Hand_high)
#     hand_mask = cv2.dilate(hand_mask, kernel = np.ones((11,11),np.uint8))
#     skin_mask = cv2.inRange(hsv, Skin_low, Skin_high)
#     skin_mask = cv2.dilate(skin_mask, kernel = np.ones((11,11),np.uint8))
#     thresh = 255 - green_mask
#     thresh = cv2.subtract(thresh, hand_mask)
#     thresh = cv2.subtract(thresh, skin_mask)
#     thresh[477:, 50:610] = 0
#     return thresh

# def get_k_dis((x1, y1), (x2, y2), (x, y)):
#         coord = ((x, y), (x1, y1), (x2, y2))
#         return Polygon(coord).area / distant((x1, y1), (x2, y2))

# class temp_tracking():
#     def __init__(self):
#         self.cap = cv2.VideoCapture(0)

#     def update(self):
#         self.surfacels = []
#         self.boxls = []
#         OK, origin = self.cap.read()
#         x = None
#         if OK:
#             rect = camrectify(origin)
#             warp = warp_img(rect)
#             thresh = get_objectmask(warp)
#             cv2.imshow('thresh', thresh)
#             self.image = warp.copy()
#             draw_img1 = warp.copy()
#             self.get_bound(draw_img1, thresh, visualization=True)
#             cx, cy = None, None
#             #print(self.surfacels)
#             (point, angle), (point2, angle2), center = hand_tracking(warp_img(origin), cache(10), cache(10)).get_result()
#             if point and len(self.boxls) > 0:
#                 length_ls = []
#                 for x, y, w, h in self.boxls:
#                     length_ls.append((get_k_dis((point[0], point[1]), (center[0], center[1]), (x+w/2, y+h/2)), (x+w/2, y+h/2)))
#                 length_ls = filter(lambda x: (point[1] - x[1][1]) * (point[1] - center[1]) <= 0, length_ls)
#                 if len(length_ls) > 0:
#                     x,y = min(length_ls, key=lambda x: x[0])[1]
#                     ind = test_insdie((x, y), self.boxls)
#                     x, y, w, h = self.boxls[ind]
#                     cx, cy = self.surfacels[ind]
#                     cv2.rectangle(draw_img1,(x,y),(x+w,y+h),(0,0,255),2)
#                     cv2.circle(draw_img1, (cx, cy), 5, (0, 0, 255), -1)
#                     cv2.putText(draw_img1,"pointed",(x,y),cv2.FONT_HERSHEY_SIMPLEX, 1.0,(0,0,255))
            
#             if point2 and len(self.boxls) > 0:
#                 boxls = deepcopy(self.boxls)
#                 length_lsr = []
#                 length_lsl = []
#                 rpoint, lpoint = point2
#                 for x, y, w, h in self.boxls:
#                     length_lsr.append((get_k_dis((rpoint[0], rpoint[1]), (center[0], center[1]), (x+w/2, y+h/2)), (x+w/2, y+h/2)))
#                 rx,ry = min(length_lsr, key=lambda x: x[0])[1]
#                 rind = test_insdie((rx, ry), self.boxls)
#                 x, y, w, h = self.boxls[rind]
#                 del boxls[rind]
#                 cv2.rectangle(draw_img1,(x,y),(x+w,y+h),(0,0,255),2)
#                 cv2.putText(draw_img1,"pointed_right",(x,y),cv2.FONT_HERSHEY_SIMPLEX, 1.0,(0,0,255))
#                 if len(boxls) > 0:
#                     for x, y, w, h in boxls:
#                         length_lsl.append((get_k_dis((lpoint[0], lpoint[1]), (center[0], center[1]), (x+w/2, y+h/2)), (x+w/2, y+h/2)))
#                     lx,ly = min(length_lsl, key=lambda x: x[0])[1]
#                     lind = test_insdie((lx, ly), boxls)
#                     x, y, w, h = boxls[lind]
#                     cv2.rectangle(draw_img1,(x,y),(x+w,y+h),(0,0,255),2)
#                     cv2.putText(draw_img1,"pointed_left",(x,y),cv2.FONT_HERSHEY_SIMPLEX, 1.0,(0,0,255))
        
#         cv2.imshow('draw', draw_img1)
#         if cx and point:
#             return [[point[0],point[1]],[cx,cy]]
#         # if x and point:
#         #     return [[point[0],point[1]],[x+w/2,y+h/2]]

#     def get_bound(self, img, object_mask, visualization=True):
#         (_,object_contours, object_hierarchy)=cv2.findContours(object_mask,cv2.RETR_TREE,cv2.CHAIN_APPROX_SIMPLE)
#         if len(object_contours) > 0:
#             for i , contour in enumerate(object_contours):
#                 area = cv2.contourArea(contour)
#                 if area>200 and area < 2000 and object_hierarchy[0, i, 3] == -1:					
#                     M = cv2.moments(contour)
#                     cx = int(M['m10']/M['m00'])
#                     cy = int(M['m01']/M['m00'])
#                     x,y,w,h = cv2.boundingRect(contour)
#                     self.boxls.append((x, y, w, h))
#         if len(self.boxls) > 0:
#             boxls_arr = np.array(self.boxls)
#             self.boxls = boxls_arr[boxls_arr[:, 0].argsort()].tolist()
#         for x, y, w, h in self.boxls:
#             sub = self.image[y:y+h, x:x+w, :]
#             hsv = cv2.cvtColor(sub,cv2.COLOR_BGR2HSV)
#             top_mask = cv2.inRange(hsv, Top_low, Top_high)
#             (_,top_contours, object_hierarchy)=cv2.findContours(top_mask,cv2.RETR_TREE,cv2.CHAIN_APPROX_SIMPLE)
#             max_area = 0
#             for i , contour in enumerate(top_contours):
#                 area = cv2.contourArea(contour)
#                 if area>max_area and object_hierarchy[0, i, 3] == -1:					
#                     M = cv2.moments(contour)
#                     cx = int(M['m10']/M['m00'])
#                     cy = int(M['m01']/M['m00'])
#                     max_area = area
#             self.surfacels.append((cx+x, cy+y))

#     def __del__(self):
#         self.cap.release()

# def callback(msg):
#     print(msg.data)
#     global voice_flag
#     voice_flag = msg.data


# if __name__ == '__main__':
#     temp = temp_tracking()
#     rospy.init_node('hand_tracking_node')
#     pub = rospy.Publisher('/target_position', Int32MultiArray, queue_size=20)
#     sub = rospy.Subscriber('/voice_command', Int32, callback)
#     ready_for_place = False
#     pos_N_cmd = []
#     while True:
#         pos = temp.update()
#         if pos:
#             if voice_flag == 1 and not ready_for_place:
#                 pos_N_cmd.append(int(pos[1][0]))
#                 pos_N_cmd.append(int(pos[1][1]))
#                 print(pos_N_cmd)
#                 ready_for_place = True
#             elif voice_flag == 2 and ready_for_place:
#                 pos_N_cmd.append(int(pos[0][0]))
#                 pos_N_cmd.append(int(pos[0][1]))
#                 print(pos_N_cmd)
#                 pub.publish(Int32MultiArray(data=pos_N_cmd))
#                 ready_for_place = False
#                 pos_N_cmd = []
#             elif voice_flag == -1:
#                 break
#         k = cv2.waitKey(1) & 0xFF # large wait time to remove freezing
#         if k == 113 or k == 27:
#             break
#     cv2.destroyAllWindows()