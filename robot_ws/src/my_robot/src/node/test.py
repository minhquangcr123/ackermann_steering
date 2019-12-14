#!/usr/bin/env python
from std_msgs.msg import Float32
import rospy
rospy.init_node("hello")
pub = rospy.Publisher("hello", Float32,queue_size = 1)
rate = rospy.Rate(10)

while not rospy.is_shutdown():
    a = Float32()
    a.data = 1
    pub.publish(a)
    rate.sleep()