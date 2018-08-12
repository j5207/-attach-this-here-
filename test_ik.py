#!/usr/bin/env python
from inverseKinematicsUR5 import InverseKinematicsUR5, transformRobotParameter
import numpy as np
from math import pi
from sensor_msgs.msg import JointState
import rospy
import tf

class ur5_joint_estimator:
	def __init__(self):
		self.tf_listener = tf.TransformListener()
		self.Transformer = tf.TransformerROS()
		self.ee_link = '/ee_link'
		self.base_link = '/base_link'
		self.ik = InverseKinematicsUR5()
		self.ik.setEERotationOffsetROS()
		self.ik.setJointWeights([6,5,4,3,2,1])
		self.ik.setJointLimits(-pi, pi)

	def estimateJointValueByUsingTF(self):
		position, quaternion = self.tf_listener.lookupTransform(self.base_link, self.ee_link,rospy.Time(0))
		transform = self.Transformer.fromTranslationRotation(position, quaternion)
		#print 'input transform:\n', transform
		joint_states = rospy.wait_for_message("joint_states", JointState)
		print 'Solutions:\n', self.ik.solveIK(transform)
		print joint_states.position

rospy.init_node('ur5_joint_estimator')
r = rospy.Rate(1)
a = ur5_joint_estimator()
while not rospy.is_shutdown():
	r.sleep()
	a.estimateJointValueByUsingTF()
	print



# 	Translation: [0.365, 0.173, 0.513]
# - Rotation: in Quaternion [0.707, 0.707, -0.001, -0.001]