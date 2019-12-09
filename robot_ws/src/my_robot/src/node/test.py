#!/usr/bin/env python
import roslib
import rospy
roslib.load_manifest('my_robot')
import numpy as np
from gazebo_msgs.msg import LinkStates
from geometry_msgs.msg import Pose
from nav_msgs.msg import Odometry
import tf

from std_msgs.msg import Header
from gazebo_msgs.srv import GetModelState, GetModelStateRequest

rospy.init_node('odom_pub')
rospy.wait_for_service ('/gazebo/get_model_state')
get_model_srv = rospy.ServiceProxy('/gazebo/get_model_state', GetModelState)

odom=Odometry()
header = Header()
header.frame_id='/odom'

model = GetModelStateRequest()
model.model_name='ackerman'
result = get_model_srv(model)
print(result)

