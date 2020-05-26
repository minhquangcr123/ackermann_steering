#!/usr/bin/env python

import rospy
import numpy as np
from nav_msgs.msg import Odometry
from obstacle_detector.msg import Obstacles,CircleObstacle,SegmentObstacle

rospy.init_node("test_publish")


pub = rospy.Publisher("/raw_obstacles", Obstacles, queue_size = 10)
rate = rospy.Rate(10.0)
obs = Obstacles()

segment = SegmentObstacle()
circle = CircleObstacle()

segment2 = SegmentObstacle()
circle2 = CircleObstacle()

segment3 = SegmentObstacle()
circle3 = CircleObstacle()

segment.first_point.x = 0.99
circle.center.x = 1

segment2.first_point.x = 0.55
segment2.first_point.y = 0.83
circle2.center.x = 1

segment3.first_point.x = 0.23
segment3.first_point.y = 0.97
circle3.center.x = 1

obs.segments = [segment, segment2,segment3] 
obs.circles = [circle, circle2, circle3]

while not rospy.is_shutdown():
  pub.publish(obs)
  rate.sleep()

