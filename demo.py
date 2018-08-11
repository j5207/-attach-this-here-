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
from std_msgs.msg import String
from geometry_msgs.msg import PoseStamped, Pose, Point, Quaternion
from moveit_msgs.msg import Grasp, GripperTranslation, PlaceLocation
from control_msgs.msg import *
from trajectory_msgs.msg import *
from sensor_msgs.msg import JointState
import actionlib
import rospy
from ros_myo.msg import EmgArray
from std_msgs.msg import String, Header
from geometry_msgs.msg import WrenchStamped, Vector3
import tf
from tf.transformations import *
from icl_phri_robotiq_control.robotiq_utils import *
from inverseKinematicsUR5 import InverseKinematicsUR5, transformRobotParameter


normalize = lambda x: x/np.sqrt(x[0]**2.+x[1]**2.+x[2]**2.)
norm = lambda a:np.sqrt(a.x**2.+a.y**2.+a.z**2.)
to_quat = lambda o: np.array([o.x, o.y, o.z, o.w])
rad_to_ang = lambda x: x / np.pi * 180.
ang_to_rad = lambda x: x / 180. * np.pi
JOINT_NAMES = ['shoulder_pan_joint', 'shoulder_lift_joint', 'elbow_joint',
               'wrist_1_joint', 'wrist_2_joint', 'wrist_3_joint']
Q1 = [0.09981511533260345, -2.024665657673971, -0.7203519980060022, -1.7111480871783655, 1.5250813961029053, 0.0007902098586782813]
Q2 = [0.09786205738782883, -2.2906211058246058, -0.7209880987750452, -1.749324146901266, 1.5192002058029175, 0.001340735238045454]
Q3 = [-1.725508991871969, -0.9326947371112269, -1.830911938344137, -0.3809617201434534, 1.7116585969924927, 0]


class Handover:
    def __init__(self):
        #/vel_based_pos_traj_controller/
        self.client = actionlib.SimpleActionClient('icl_phri_ur5/follow_joint_trajectory', FollowJointTrajectoryAction)
        self.goal = FollowJointTrajectoryGoal()
        self.goal.trajectory = JointTrajectory()
        self.goal.trajectory.joint_names = JOINT_NAMES
        print ("Waiting for server...")
        self.client.wait_for_server()
        print ("Connected to server")
        joint_states = rospy.wait_for_message("icl_phri_ur5/joint_states", JointState)
        self.joints_pos_start = np.array(joint_states.position)
        print ("Init done")
        self.listener = tf.TransformListener()

    def move(self):
        current_m = transformRobotParameter(self.joints_pos_start)
        hand_base = self._get_transform('/ee_link', '/right_hand_1')
        gripper_hand = None
        Desti_m = self._cal_dest(gripper_hand, hand_base)
        sols = inverseinverse(np.array(Desti_m), float(self.joints_pos_start[5]))
        qsol = best_sol(sols, self.joints_pos_start, [1.]*6)
        if qsol is not None:
            self.goal.trajectory.points = [
                JointTrajectoryPoint(positions=self.joints_pos_start.tolist(), velocities=[0]*6, time_from_start=rospy.Duration(0.0)),
                JointTrajectoryPoint(positions=qsol.tolist(), velocities=[0]*6, time_from_start=rospy.Duration(1./125.)),
            ]
            print('start: ' + str(self.joints_pos_start.tolist()))
            print('goal: ' + str(qsol.tolist()))
            try:
                self.client.send_goal(self.goal)
                self.joints_pos_start = qsol
            except:
                raise
    
    def _get_transform(self, target_frame, source_frame, is_matrix=True):
        rate = rospy.Rate(10.0)
        rate.sleep()        
        # try:
        (trans, quat) = self.listener.lookupTransform(target_frame, source_frame, rospy.Time(0))
        if is_matrix:
            rot = euler_from_quaternion(quat)            
            return np.asarray(tf.TransformerROS.fromTranslationRotation(trans, rot))
        else:
            return trans, quat
        # except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
        #     print("wait for tf")
    
    def _cal_dest(self, gripper_hand, hand_base):
        return np.dot(gripper_hand , hand_base)
        







if __name__ == '__main__':
    rospy.init_node('test', anonymous=True)
    h = Handover()
    rospy.sleep(1)
    while not rospy.is_shutdown():
        try:
            # rospy.sleep(1)
            trans = h._get_transform('/left_hand_1', '/right_hand_1')[0]
            # print("hand_gripper_tf_trans:{}\nhand_gripper_tf_quat:{}".format(
            #     h._get_transform('/ee_link', '/right_hand_1')[0],
            #     h._get_transform('/ee_link', '/right_hand_1')[1]
            # ))
            # print(trans)
            h.frame_action(trans)
        except e:
            print(e)


