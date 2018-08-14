#!/usr/bin/env python
from __future__ import print_function
import sys
import rospy
import moveit_commander
from moveit_commander import RobotCommander, MoveGroupCommander
from moveit_commander import PlanningSceneInterface, roscpp_initialize, roscpp_shutdown
from geometry_msgs.msg import PoseStamped
from moveit_msgs.msg import Grasp, GripperTranslation, PlaceLocation
from trajectory_msgs.msg import JointTrajectoryPoint
import tf
from copy import deepcopy
from icl_phri_robotiq_control.robotiq_utils import *
from std_msgs.msg import Int32MultiArray, Int32
import cv2

def define_grasp(position):
        grasp_pose = PoseStamped()
        quat = tf.transformations.quaternion_from_euler(-3.14, 0, 0)
        grasp_pose.header.frame_id = "/base_link"
        grasp_pose.pose.position.x = position[0]
        grasp_pose.pose.position.y = position[1]
        grasp_pose.pose.position.z = position[2]
        grasp_pose.pose.orientation.x = quat[0]
        grasp_pose.pose.orientation.y = quat[1]
        grasp_pose.pose.orientation.z = quat[2]
        grasp_pose.pose.orientation.w = quat[3]
        return grasp_pose

def coord_converter(x, y):
    pts1 = np.array([[138, 133], [281, 133], [429, 130], [134, 271], [280, 251], [417, 260], [137, 391], [274, 381]])
    pts2 = np.array([[0.391, -0.319], [0.557, -0.337], [0.719, -0.355], [0.386, -0.496], [0.557, -0.489], [0.707, -0.470], [0.379, -0.640], [0.541, -0.641]])
    M, mask = cv2.findHomography(pts1, pts2, cv2.RANSAC,5.0)
    solusion = np.matmul(M,np.array([x, y, 1]))
    solusion = solusion/solusion[2]
    return solusion[0], solusion[1]

class pick_place():
    def __init__(self):
        moveit_commander.roscpp_initialize(sys.argv)
        self.scene = PlanningSceneInterface()
        self.robot = RobotCommander()
        self.ur5 = MoveGroupCommander("manipulator")
        self.gripper = MoveGroupCommander("end_effector")
        print("======================================================")
        print(self.robot.get_group_names())
        print(self.robot.get_link_names())
        print(self.robot.get_joint_names())
        print(self.robot.get_planning_frame())
        print(self.ur5.get_end_effector_link())
        print("======================================================")
        self.ur5.set_max_velocity_scaling_factor(0.1)
        self.ur5.set_max_acceleration_scaling_factor(0.1)
        self.ur5.set_end_effector_link("fts_toolside")
        self.ur5.set_planning_time(60.0)
        #self.ur5.set_planner_id("RRTkConfigDefault")
        self.gripper_ac = RobotiqActionClient('icl_phri_gripper/gripper_controller')
        self.gripper_ac.wait_for_server()
        self.gripper_ac.initiate()

        self.sub = rospy.Subscriber('/target_position', Int32MultiArray, self.pickplace_cb)


    def single_exuete(self, position, mode):
        offset = 0.02
        rospy.loginfo("let do a single exuete")
        rospy.sleep(5)
        position_copy = deepcopy(position)
        position_copy += [0.14]
        position_copy[1] = position_copy[1] - offset
        pre_position = define_grasp([position_copy[0], position_copy[1], position_copy[2] + 0.2])
        post_position = define_grasp([position_copy[0], position_copy[1], position_copy[2] + 0.2])
        grasp_position = define_grasp(position_copy)
        self.ur5.set_pose_target(pre_position)
        self.ur5.go()
        self.ur5.stop()
        self.ur5.clear_pose_targets()
        rospy.sleep(5)
        self.ur5.set_pose_target(grasp_position)
        self.ur5.go()
        self.ur5.stop()
        self.ur5.clear_pose_targets()
        if mode == "pick":
            self.gripper_ac.send_goal(0)
        if mode == "place":
            self.gripper_ac.send_goal(0.14)
        self.gripper_ac.wait_for_result()
        rospy.sleep(5)
        self.ur5.set_pose_target(post_position)
        self.ur5.go()
        self.ur5.stop()
        self.ur5.clear_pose_targets()
        rospy.sleep(5)

    def pair_exuete(self, pick_position, place_position):
        rospy.loginfo("here we go pair")
        if pick_position and place_position:
            self.single_exuete(pick_position, "pick")
            self.single_exuete(place_position, "place")
            rospy.sleep(5)
            rospy.loginfo("let's go and get some rest")
            rest_position = define_grasp([0.486, -0.152, 0.342])
            self.ur5.set_pose_target(rest_position)
            self.ur5.go()
            self.ur5.stop()
            self.ur5.clear_pose_targets()

    def pickplace_cb(self, msg):
        #print(msg)
        print(msg.data)
        rospy.loginfo("oh my god, that's a callback! something gonna happen")
        pick_x, pick_y = coord_converter(msg.data[0], msg.data[1])
        place_x, place_y = coord_converter(msg.data[2], msg.data[3])
        print(pick_x, pick_y)
        print(place_x, place_y)
        self.pair_exuete([pick_x, pick_y], [place_x, place_y])


# class exuete():
#     def __init__(self, task, sub):
#         self.task = task
#         self.target_sub = sub
    
#     def pickplace_cb(self, msg):
#         #print(msg)
#         print(msg.data)
#         pick_x, pick_y = coord_converter(msg.data[0], msg.data[1])
#         place_x, place_y = coord_converter(msg.data[2], msg.data[3])
#         print(pick_x, pick_y)
#         print(place_x, place_y)
#         self.task.pair_exuete([pick_x, pick_y], [place_x, place_y])
        

    


if __name__=='__main__':
    rospy.init_node('pick_place', anonymous=True)
    task = pick_place()
    rospy.spin()
    # sub = rospy.Subscriber('/target_position', Int32MultiArray, pickplace_cb)
    # while True:
    #     #print("g")
    #     exuete(task, sub)
    #     if not rospy.is_shutdown:
    #         break
    # task.pair_exuete([0.96, -0.7], [0.558, -0.35])
 
