#!/usr/bin/env python
import cv2
import numpy as np
import tf
import rospy, math
from geometry_msgs.msg import PoseStamped
import actionlib 
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from geometry_msgs.msg import Pose, Point, Quaternion
from tf.transformations import quaternion_from_euler
from sympy import symbols, Eq, solve
import argparse
from obstacle_detector.msg import Obstacles,CircleObstacle,SegmentObstacle
from gazebo_msgs.srv import GetModelState, GetModelStateRequest
import message_filters
from sensor_msgs.msg import LaserScan
import matplotlib.pyplot as plt
import sys
import roslaunch
from geometry_msgs.msg import Twist
from gazebo_msgs.srv import GetModelState, GetModelStateRequest
from nav_msgs.msg import Odometry
from std_msgs.msg import Header
from gazebo_msgs.msg import ModelStates

def call_back(data):
    # twist = data.twist[12]
    v_x = data.twist.twist.linear.x
    v_y = data.twist.twist.linear.y
    v_angular = data.twist.twist.angular.z
    v_linear = np.sqrt(v_x ** 2 + v_y ** 2 ) * 1
    velocity.linear.x = v_linear
    velocity.angular.z = v_angular
    pub_path.publish(velocity)  

rospy.init_node("test_node")

rate = rospy.Rate(10.0)
pub_path = rospy.Publisher("visualize_gazebo_velocity", Twist, queue_size = 1 )

begin_time = 0
count = 0
time_move = 5
epsilon = 0.05
velocity = Twist()


rospy.Subscriber("/odom", Odometry, call_back)
rospy.spin()

