#!/usr/bin/env python
import rospy
from std_msgs.msg import Int32


rospy.init_node('fake_publisher')
pub = rospy.Publisher('/voice_command1', Int32, queue_size=1)
while True:
    num = raw_input('please enter \n')
    pub.publish(Int32(data=int(num)))
    if not rospy.is_shutdown:
        break
#rospy.spin()