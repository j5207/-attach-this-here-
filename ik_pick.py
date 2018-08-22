#!/usr/bin/env python

from __future__ import division, print_function
import numpy as np
from math import *
from time import sleep
import sys
import copy
import moveit_commander
import moveit_msgs.msg
import geometry_msgs.msg
import roslib
from std_msgs.msg import String, Int32MultiArray
from geometry_msgs.msg import PoseStamped, Pose, Point, Quaternion
from moveit_msgs.msg import Grasp, GripperTranslation, PlaceLocation
from control_msgs.msg import *
from trajectory_msgs.msg import *
from sensor_msgs.msg import JointState
import actionlib
import rospy
from std_msgs.msg import String, Header
from geometry_msgs.msg import WrenchStamped, Vector3
import tf
from tf.transformations import *
from math import pi
# from icl_phri_robotiq_control.robotiq_utils import *
from inverseKinematicsUR5 import InverseKinematicsUR5, transformRobotParameter
from copy import deepcopy
import cv2


JOINT_NAMES = ['shoulder_pan_joint', 'shoulder_lift_joint', 'elbow_joint',
               'wrist_1_joint', 'wrist_2_joint', 'wrist_3_joint']


def coord_converter(x, y):
    # pts1 = np.array([[138, 133], [281, 133], [429, 130], [134, 271], [280, 251], [417, 260], [137, 391], [274, 381]])
    # pts2 = np.array([[0.391, -0.319], [0.557, -0.337], [0.719, -0.355], [0.386, -0.496], [0.557, -0.489], [0.707, -0.470], [0.379, -0.640], [0.541, -0.641]])
    pts1 = np.array([[(118, 84), (131, 239), (134, 369), (295, 354), (292, 85), (304, 227), (444, 237), (465, 76)]])
    pts2 = np.array([[(0.369, -0.309), (0.399, -0.497), (0.376, -0.646), (0.589, -0.643), (0.568, -0.310), (0.589, -0.495),(0.759, -0.495), (0.792, -0.301)]])
    M, mask = cv2.findHomography(pts1, pts2, cv2.RANSAC,5.0)
    solusion = np.matmul(M,np.array([x, y, 1]))
    solusion = solusion/solusion[2]
    return solusion[0], solusion[1]
#tool0 is ee
class pick_place:
    def __init__(self):
        #/vel_based_pos_traj_controller/
        self.client = actionlib.SimpleActionClient('arm_controller/follow_joint_trajectory', FollowJointTrajectoryAction)
        self.goal = FollowJointTrajectoryGoal()
        self.goal.trajectory = JointTrajectory()
        self.goal.trajectory.joint_names = JOINT_NAMES
        print ("Waiting for server...")
        self.client.wait_for_server()
        print ("Connected to server")
        joint_states = rospy.wait_for_message("joint_states", JointState)
        self.joints_pos_start = np.array(joint_states.position)
        print ("Init done")
        self.listener = tf.TransformListener()
        self.Transformer = tf.TransformerROS()

        joint_weights = [10,5,4,3,2,1]
        self.ik = InverseKinematicsUR5()
        self.ik.setJointWeights(joint_weights)
        self.ik.setJointLimits(-pi, pi)
        self.sub = rospy.Subscriber('/target_position', Int32MultiArray, self.pickplace_cb)

        # self.gripper_ac = RobotiqActionClient('icl_phri_gripper/gripper_controller')
        # self.gripper_ac.wait_for_server()
        # self.gripper_ac.initiate()
        # self.gripper_ac.send_goal(0.08)
        # self.gripper_ac.wait_for_result()

    def define_grasp(self, position):
        quat = tf.transformations.quaternion_from_euler(3.14, 0, -3.14)
        dest_m = self.Transformer.fromTranslationRotation(position, quat) 
        return dest_m

    def move(self, dest_m):
        #current_m = transformRobotParameter(self.joints_pos_start)
        qsol = self.ik.findClosestIK(dest_m,self.joints_pos_start)
        
        if qsol is not None:
            if qsol[0] < 0:
                qsol[0] += pi
            else:
                qsol[0] -= pi
            self.goal.trajectory.points = [
                JointTrajectoryPoint(positions=self.joints_pos_start.tolist(), velocities=[0]*6, time_from_start=rospy.Duration(0.0)),
                JointTrajectoryPoint(positions=qsol.tolist(), velocities=[0]*6, time_from_start=rospy.Duration(6)),
            ]
            print('start: ' + str(self.joints_pos_start.tolist()))
            print('goal: ' + str(qsol.tolist()))
            try:
                self.client.send_goal(self.goal)
                self.joints_pos_start = qsol
                self.client.wait_for_result()
            except:
                raise
        else:
            rospy.loginfo("fail to find IK solution")
    
    def single_exuete(self, position, mode):
        offset = 0.01
        position_copy = deepcopy(position)
        position_copy += [0.14]
        position_copy[1] = position_copy[1] + offset
        pre_position = self.define_grasp([position_copy[0], position_copy[1], position_copy[2] + 0.2])
        post_position = self.define_grasp([position_copy[0], position_copy[1], position_copy[2] + 0.2])
        grasp_position = self.define_grasp(position_copy)
        rospy.loginfo("let's go to the pre location")
        self.move(pre_position)
        rospy.sleep(1)
        rospy.loginfo("let's do this")
        self.move(grasp_position)
        rospy.sleep(1)
        # if mode == "pick":
        #     self.gripper_ac.send_goal(0)
        # elif mode == "place":
        #     self.gripper_ac.send_goal(0.08)
        # self.gripper_ac.wait_for_result()
        rospy.loginfo("move out")
        self.move(post_position)
        rospy.sleep(1)

    def pair_exuete(self, pick_position, place_position):
        rospy.loginfo("here we go pair")
        if pick_position and place_position:
            self.single_exuete(pick_position, "pick")
            self.single_exuete(place_position, "place")
            #rospy.sleep(1)
            rospy.loginfo("let's go and get some rest")
            rest_position = self.define_grasp([0.486, -0.152, 0.342])
            self.move(rest_position)
            rospy.sleep(1)
    
    def pickplace_cb(self, msg):
        #print(msg)
        print(msg.data)
        a = list(msg.data)
        mean_x = np.mean([a[i] for i in range(0, len(a)-2, 2)])
        mean_y = np.mean([a[i] for i in range(1, len(a)-2, 2)])
        num_goals = (len(msg.data) -2)/2
        rospy.loginfo("there is {} goals".format(num_goals))
        for i in range(0, len(a)-2, 2):
            pick_x, pick_y = coord_converter(msg.data[i], msg.data[i+1])
            leeway_x = int(msg.data[i] - mean_x)
            leeway_y = int(msg.data[i+1] - mean_y)
            place_x, place_y = coord_converter(msg.data[-2] + leeway_x, msg.data[-1] + leeway_y)
            print(pick_x, pick_y)
            print(place_x, place_y)
            self.pair_exuete([pick_x, pick_y], [place_x, place_y])







if __name__ == '__main__':
    rospy.init_node('test', anonymous=True)
    task = pick_place()
    # pick_x, pick_y = coord_converter(108, 150)
    # place_x, place_y = coord_converter(177, 193)
    pick_x, pick_y = coord_converter(640, 480)
    place_x, place_y = coord_converter(0, 0)
    task.pair_exuete([pick_x, pick_y], [place_x, place_y])
    # listener = tf.TransformListener()
    # Transformer = tf.TransformerROS()
    # try:
    #     listener.waitForTransform('/base_link','/target',rospy.Time(), rospy.Duration(4.0))
    #     position, quaternion = listener.lookupTransform('/base_link', '/target',rospy.Time(0))
    #     transform = Transformer.fromTranslationRotation(position, quaternion) 
    #     h.move(transform)
    # except Exception as e:
    #     print(e)
    # rospy.spin()
    # while not rospy.is_shutdown():
    #     try:
            
    #     except e:
    #         print(e)


