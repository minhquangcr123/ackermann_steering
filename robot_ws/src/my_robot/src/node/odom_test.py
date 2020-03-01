#!/usr/bin/env python

import rospy
import numpy as np
from nav_msgs.msg import Odometry
from point import goal
rospy.init_node("test_publish")
pub = rospy.Publisher("/my_odom", Odometry, queue_size = 10)
rate = rospy.Rate(10.0)
odom = Odometry()
odom.pose.pose.position.x =1
odom.pose.pose.position.y = 1
#while not rospy.is_shutdown():
  #  pub.publish(odom)
   # rate.sleep()
print(goal)