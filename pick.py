#!/usr/bin/env python

import sys
import rospy
from moveit_commander import RobotCommander, MoveGroupCommander
from moveit_commander import PlanningSceneInterface, roscpp_initialize, roscpp_shutdown
from geometry_msgs.msg import PoseStamped
from moveit_msgs.msg import Grasp, GripperTranslation, PlaceLocation
from trajectory_msgs.msg import JointTrajectoryPoint
import tf
import copy
from moveit_python import pick_place_interface
from copy import deepcopy

def make_gripper_posture(pose, gripper_joint_names):
        t = JointTrajectory()
        t.joint_names = gripper_joint_names
        tp = JointTrajectoryPoint()
        tp.positions = [pose/2.0 for j in t.joint_names]
        tp.effort = gripper_effort
        t.points.append(tp)
        return t

def make_gripper_translation(min_dist, GRIPPER_FRAME, desired, axis=1.0):
    g = GripperTranslation()
    g.direction.vector.x = axis
    g.direction.header.frame_id = GRIPPER_FRAME
    g.min_distance = min_dist
    g.desired_distance = desired
    return g

def make_grasps(pose_stamped, mega_angle=False):
    # setup defaults of grasp
    g = Grasp()
    g.pre_grasp_posture = make_gripper_posture(GRIPPER_OPEN)
    g.grasp_posture = make_gripper_posture(GRIPPER_CLOSED)
    g.pre_grasp_approach = make_gripper_translation(0.1, 0.15)
    g.post_grasp_retreat = make_gripper_translation(0.1, 0.15, -1.0)
    g.grasp_pose = pose_stamped

    pitch_vals = [0, 0.2, -0.2, 0.4, -0.4]
    if mega_angle:
        pitch_vals += [0.3, -0.3, 0.5, -0.5, 0.6, -0.6]

    # generate list of grasps
    grasps = []
    for y in [-1.57, -0.78, 0, 0.78, 1.57]:
        for p in pitch_vals:
            q = quaternion_from_euler(0, 1.57-p, y)
            g.grasp_pose.pose.orientation.x = q[0]
            g.grasp_pose.pose.orientation.y = q[1]
            g.grasp_pose.pose.orientation.z = q[2]
            g.grasp_pose.pose.orientation.w = q[3]
            g.id = str(len(grasps))
            g.grasp_quality = 1.0 - abs(p/2.0)
            grasps.append(copy.deepcopy(g))
    return grasps

def make_places(self, pose_stamped, mega_angle=False):
    # setup default of place location
    l = PlaceLocation()
    l.post_place_posture = self.make_gripper_posture(GRIPPER_OPEN)
    l.pre_place_approach = self.make_gripper_translation(0.1, 0.15)
    l.post_place_retreat = self.make_gripper_translation(0.1, 0.15, -1.0)
    l.place_pose = pose_stamped

    pitch_vals = [0, 0.2, -0.2, 0.4, -0.4]
    if mega_angle:
        pitch_vals += [0.3, -0.3, 0.5, -0.5, 0.6, -0.6]

    # generate list of place locations
    places = []
    for y in [-1.57, -0.78, 0, 0.78, 1.57]:
        for p in pitch_vals:
            q = quaternion_from_euler(0, p, y)  # now in object frame
            l.place_pose.pose.orientation.x = q[0]
            l.place_pose.pose.orientation.y = q[1]
            l.place_pose.pose.orientation.z = q[2]
            l.place_pose.pose.orientation.w = q[3]
            l.id = str(len(places))
            places.append(copy.deepcopy(l))
    return places
if __name__=='__main__':

    roscpp_initialize(sys.argv)
    rospy.init_node('moveit_py_demo', anonymous=True)
    
    scene = PlanningSceneInterface()
    robot = RobotCommander()
    right_arm = MoveGroupCommander("manipulator")
    right_arm.set_end_effector_link("ee_link")
    right_arm.set_planning_time(30.0)
    #right_arm.setPlannerId("RRTConnectkConfigDefault")
    #right_arm.set_planner_id("RRTConnectkConfigDefault")
    # right_gripper = MoveGroupCommander("right_gripper")
    rospy.sleep(1)

    # clean the scene
    #scene.remove_world_object("table")
    scene.remove_world_object("part")

    # # right_arm.set_named_target("resting")
    # # right_arm.go()
    
    # #right_gripper.set_named_target("open")
    # #right_gripper.go()
    
    # rospy.sleep(1)

    # # publish a demo scene
    p = PoseStamped()
    p.header.frame_id = robot.get_planning_frame()

    # add a table
    # p.pose.position.x = 0.2
    # p.pose.position.y = -0.2
    # p.pose.position.z = 0.3
    # scene.add_box("table", p, (0.1, 0.5, 0.2))

    # add an object to be grasped
    p.pose.position.x = 0.205
    p.pose.position.y = -0.12
    p.pose.position.z = 0.3
    scene.add_box("part", p, (0.02, 0.001, 0.02))
    
    rospy.sleep(1)
    
    grasps = []
    # lisener = tf.TransformListener()
    # lisener.waitForTransform('/base_link', '/target', rospy.Time(),rospy.Duration(4.0))
    # translation, quaternion = lisener.lookupTransform('/base_link', '/target', rospy.Time(0))
    
    g = Grasp()
    g.id = "test"
    grasp_pose = PoseStamped()
    grasp_pose.header.frame_id = "base_link"
    grasp_pose.pose.position.x = 0.138554
    grasp_pose.pose.position.y = -0.116075
    grasp_pose.pose.position.z = 0.70493
    grasp_pose.pose.orientation.x = -0.709103
    grasp_pose.pose.orientation.y = 0.0137777
    grasp_pose.pose.orientation.z = 0.0164031
    grasp_pose.pose.orientation.w = 0.704779
    
    # right_arm.set_pose_target(grasp_pose)
    # right_arm.go()

    grasp_pose1 = deepcopy(grasp_pose)
    grasp_pose1.pose.position.x += 0.01

    grasp_pose2 = deepcopy(grasp_pose)
    right_arm.set_pose_target(grasp_pose)
    right_arm.go()
    right_arm.stop()
    rospy.sleep(5)
    right_arm.set_pose_target(grasp_pose1)
    right_arm.go()
    right_arm.stop()
    rospy.sleep(5)
    right_arm.set_pose_target(grasp_pose2)
    right_arm.go()
    #rospy.sleep(10)

    # print "grasp pose"
    # rospy.sleep(8)
    
    # # # set the grasp pose
    # grasp_pose = PoseStamped()
    # grasp_pose.header.frame_id = "ee_link"
    # grasp_pose.pose.position.x = 0.148554
    # grasp_pose.pose.position.y = -0.116075
    # grasp_pose.pose.position.z = 0.70493
    # grasp_pose.pose.orientation.x = -0.709103
    # grasp_pose.pose.orientation.y = 0.0137777
    # grasp_pose.pose.orientation.z = 0.0164031
    # grasp_pose.pose.orientation.w = 0.704779
    # g.grasp_pose = grasp_pose
    
    # # define the pre-grasp approach
    # g.pre_grasp_approach.direction.header.frame_id = "base_link"
    # g.pre_grasp_approach.direction.vector.x = 1.0
    # g.pre_grasp_approach.direction.vector.y = 0.0
    # g.pre_grasp_approach.direction.vector.z = 0.0
    # g.pre_grasp_approach.min_distance = 0.009
    # g.pre_grasp_approach.desired_distance = 0.01
    
    # g.pre_grasp_posture.header.frame_id = "wrist_3_link"
    # g.pre_grasp_posture.joint_names = ["wrist_3_joint"]
    
    # pos = JointTrajectoryPoint()
    # pos.positions.append(0.8)
    # pos.effort.append(0.0) 
    # g.pre_grasp_posture.points.append(pos)
    
    # # set the grasp posture
    # g.grasp_posture.header.frame_id = "wrist_3_link"
    # g.grasp_posture.joint_names = ["wrist_3_joint"]

    # pos = JointTrajectoryPoint()
    # pos.positions.append(0.2)
    # pos.effort.append(0.0) 
    # g.grasp_posture.points.append(pos)

    # # pos = JointTrajectoryPoint()
    # # pos.positions.append(0.3)
    # # pos.effort.append(0.0) 
    # # g.grasp_posture.points.append(pos)

    # # pos = JointTrajectoryPoint()
    # # pos.positions.append(0.4)
    # # pos.effort.append(0.0) 
    # # g.grasp_posture.points.append(pos)

    # # pos = JointTrajectoryPoint()
    # # pos.positions.append(0.5)
    # # pos.effort.append(0.0) 
    # # g.grasp_posture.points.append(pos)

    # # pos = JointTrajectoryPoint()
    # # pos.positions.append(0.6)
    # # pos.effort.append(0.0) 
    # # g.grasp_posture.points.append(pos)
    
   
    

    # # set the post-grasp retreat
    # g.post_grasp_retreat.direction.header.frame_id = "base_link"
    # g.post_grasp_retreat.direction.vector.x = -1.0
    # g.post_grasp_retreat.direction.vector.y = 0.0
    # g.post_grasp_retreat.direction.vector.z = 0.0
    # g.post_grasp_retreat.desired_distance = 0.01
    # g.post_grasp_retreat.min_distance = 0.009

    # g.allowed_touch_objects = ["part"]

    # g.max_contact_force = 0
    
    # # append the grasp to the list of grasps
    # for y in [-1.57, -0.78, 0, 0.78, 1.57]:
    #     for p in [0, 0.2, -0.2, 0.4, -0.4]:
    #         q = tf.transformations.quaternion_from_euler(0, 1.57-p, y)
    #         g.grasp_pose.pose.orientation.x = q[0]
    #         g.grasp_pose.pose.orientation.y = q[1]
    #         g.grasp_pose.pose.orientation.z = q[2]
    #         g.grasp_pose.pose.orientation.w = q[3]
    #         g.id = str(len(grasps))
    #         g.grasp_quality = 1.0 - abs(p/2.0)
    #         grasps.append(copy.deepcopy(g))
    # rospy.sleep(2)
    # #print grasps
    # # pick the object
    # right_arm.pick("part", grasps)

    # rospy.spin()
    # roscpp_shutdown()
