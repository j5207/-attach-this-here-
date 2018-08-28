import cv2
import numpy as np
from constant import *
from pyemd import emd_samples
from math import sqrt, atan2
img1 = cv2.imread('/home/intuitivecompting/catkin_ws/src/ur5/ur5_with_gripper/icl_phri_robotiq_control/src/image/1.jpg')
img2 = cv2.imread('/home/intuitivecompting/catkin_ws/src/ur5/ur5_with_gripper/icl_phri_robotiq_control/src/image/1850.jpg')
img3 = cv2.imread('/home/intuitivecompting/catkin_ws/src/ur5/ur5_with_gripper/icl_phri_robotiq_control/src/image/2215.jpg')
img4 = cv2.imread('/home/intuitivecompting/catkin_ws/src/ur5/ur5_with_gripper/icl_phri_robotiq_control/src/image/7767.jpg')

distant = lambda (x1, y1), (x2, y2) : sqrt((x1 - x2)**2 + (y1 - y2)**2)

def rotate_image(mat, angle):
    # angle in degrees
    height, width = mat.shape[:2]
    image_center = (width/2, height/2)

    rotation_mat = cv2.getRotationMatrix2D(image_center, angle, 1.)

    abs_cos = abs(rotation_mat[0,0])
    abs_sin = abs(rotation_mat[0,1])

    bound_w = int(height * abs_sin + width * abs_cos)
    bound_h = int(height * abs_cos + width * abs_sin)

    rotation_mat[0, 2] += bound_w/2 - image_center[0]
    rotation_mat[1, 2] += bound_h/2 - image_center[1]
    #rotated_pst = np.matmul(rotation_mat, np.array((list(point) + [1])))
    rotated_mat = cv2.warpAffine(mat, rotation_mat, (bound_w, bound_h))
    
    return rotated_mat

def self_rotate(img):
    hsv = cv2.cvtColor(img,cv2.COLOR_BGR2HSV)
    hand_mask = cv2.inRange(hsv, Hand_low, Hand_high)
    hand_mask = cv2.dilate(hand_mask, kernel = np.ones((3,3),np.uint8))
    (_,object_contours, object_hierarchy)=cv2.findContours(hand_mask,cv2.RETR_TREE,cv2.CHAIN_APPROX_SIMPLE)
    cnt = max(object_contours, key=lambda x: cv2.contourArea(x))
    epsilon = 0.001*cv2.arcLength(cnt,True)
    cnt = cv2.approxPolyDP(cnt,epsilon,True)


    (x,y),(MA,ma),angle = cv2.fitEllipse(cnt)
    img = rotate_image(img, angle)
    return img



def get_cnt(img):
    #print(img.shape)
    img = self_rotate(img)
    hsv = cv2.cvtColor(img,cv2.COLOR_BGR2HSV)
    hand_mask = cv2.inRange(hsv, Hand_low, Hand_high)
    hand_mask = cv2.dilate(hand_mask, kernel = np.ones((3,3),np.uint8))
    (_,object_contours, object_hierarchy)=cv2.findContours(hand_mask,cv2.RETR_TREE,cv2.CHAIN_APPROX_SIMPLE)
    cnt = max(object_contours, key=lambda x: cv2.contourArea(x))
    epsilon = 0.001*cv2.arcLength(cnt,True)
    approx = cv2.approxPolyDP(cnt,epsilon,True)
    return approx



def get_value(img, frame):
    contour1, cnt = get_cnt(img.copy()), get_cnt(frame.copy())
    M = cv2.moments(contour1)
    cx = int(M['m10']/M['m00'])
    cy = int(M['m01']/M['m00'])
    dist1 = np.array(map(lambda x: distant((cx, cy), (x[0][0], x[0][1])), contour1))

    M = cv2.moments(cnt)
    cx = int(M['m10']/M['m00'])
    cy = int(M['m01']/M['m00'])
    dist2 = np.array(map(lambda x: distant((cx, cy), (x[0][0], x[0][1])), cnt))

    v = emd_samples(dist1, dist2, bins=2)
    return v

ls = [img1, img2, img3,img4]

def classifier(image,frame, box):
    x,y,w,h = box
    img =frame[y:y+h, x:x+w, :].copy()
    global ls
    d = min(ls, key=lambda x: get_value(x, img))
    label = ls.index(d) + 1
    cv2.rectangle(image,(x,y),(x+w,y+h),(0,0,255),2)
    cv2.putText(image,str(label),(x,y),cv2.FONT_HERSHEY_SIMPLEX, 1.0,(0,0,255))
    return label


def get_sold(img):
    cnt = get_cnt(img)
    area = cv2.contourArea(cnt)
    hull = cv2.convexHull(cnt)
    hull_area = cv2.contourArea(hull)
    solidity = float(area)/hull_area
    return solidity
# print(get_value(img1, img2))
# print(get_value(img1, img1))







