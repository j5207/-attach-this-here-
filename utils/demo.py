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
from std_msgs.msg import String, Header
from geometry_msgs.msg import WrenchStamped, Vector3
import tf
from tf.transformations import *
from math import pi
# from icl_phri_robotiq_control.robotiq_utils import *
from inverseKinematicsUR5 import InverseKinematicsUR5, transformRobotParameter


normalize = lambda x: x/np.sqrt(x[0]**2.+x[1]**2.+x[2]**2.)
norm = lambda a:np.sqrt(a.x**2.+a.y**2.+a.z**2.)
to_quat = lambda o: np.array([o.x, o.y, o.z, o.w])
rad_to_ang = lambda x: x / np.pi * 180.
ang_to_rad = lambda x: x / 180. * np.pi
JOINT_NAMES = ['shoulder_pan_joint', 'shoulder_lift_joint', 'elbow_joint',
               'wrist_1_joint', 'wrist_2_joint', 'wrist_3_joint']


class IK_solver:
    def __init__(self):
        #/vel_based_pos_traj_controller/
        # icl_phri_ur5
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
        # self.listener = tf.TransformListener()
        # self.Transformer = tf.TransformerROS()

        joint_weights = [6,5,4,3,2,1]
        self.ik = InverseKinematicsUR5()
        self.ik.setJointWeights(joint_weights)
        #self.ik.setJointLimits(-pi, pi)


    def move(self, dest_m):
        current_m = transformRobotParameter(self.joints_pos_start)
        qsol = self.ik.findClosestIK(dest_m,self.joints_pos_start)
        #qsol[0] -= pi
        if qsol[0] < 0:
            qsol[0] += pi
        else:
            qsol[0] -= pi
        if qsol is not None:
            self.goal.trajectory.points = [
                JointTrajectoryPoint(positions=self.joints_pos_start.tolist(), velocities=[0]*6, time_from_start=rospy.Duration(0.0)),
                JointTrajectoryPoint(positions=qsol.tolist(), velocities=[0]*6, time_from_start=rospy.Duration(10)),
            ]
            print('start: ' + str(self.joints_pos_start.tolist()))
            print('goal: ' + str(qsol.tolist()))
            try:
                self.client.send_goal(self.goal)
                self.joints_pos_start = qsol
            except:
                raise
        







if __name__ == '__main__':
    rospy.init_node('test', anonymous=True)
    h = IK_solver()
    listener = tf.TransformListener()
    Transformer = tf.TransformerROS()
    try:
        listener.waitForTransform('/base_link','/target',rospy.Time(), rospy.Duration(4.0))
        position, quaternion = listener.lookupTransform('/base_link', '/target',rospy.Time(0))
        transform = Transformer.fromTranslationRotation(position, quaternion) 
        h.move(transform)
    except Exception as e:
        print(e)
    rospy.spin()
    # while not rospy.is_shutdown():
    #     try:
            
    #     except e:
    #         print(e)


