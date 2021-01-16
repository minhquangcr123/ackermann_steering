#!/usr/bin/env python
import rospy, math
from geometry_msgs.msg import Twist
import tf
from geometry_msgs.msg import PoseStamped
import actionlib 
from geometry_msgs.msg import Pose, Point, Quaternion
from nav_msgs.msg import Path
from nav_msgs.msg import Odometry
from gazebo_msgs.srv import GetModelState, GetModelStateRequest
from std_msgs.msg import Header
import numpy as np
from sympy import symbols, Eq, solve
import time
def take_odomemetry():
    rospy.wait_for_service("/gazebo/get_model_state")
    get_model_respone = rospy.ServiceProxy("gazebo/get_model_state", GetModelState)
        
    header = Header()
    header.frame_id = "/odom"
    header.stamp = rospy.Time.now()

    model = GetModelStateRequest()
    model.model_name = "ackerman"
    result = get_model_respone(model)
        
    odom = Odometry()
    odom.header = header
    odom.pose.pose = result.pose
    odom.twist.twist = result.twist
    quat = [0,0, odom.pose.pose.orientation.z, odom.pose.pose.orientation.w]
    quat = quat / np.linalg.norm(quat)
    odom_angle = tf.transformations.euler_from_quaternion([0, 0, quat[2], quat[3]])
    return [odom.pose.pose.position.x, odom.pose.pose.position.y, odom_angle[2]]

class move_parking():
    def __init__(self):
        rospy.init_node("move_parking") 
        self.wheel_base = 0.335004153719 # wheel base distance
        self.theta = math.pi / 10
        self.velocity_linear = 0.03
        self.move_point = []
        self.center_point = []
        self.goal_point = []
        self.rate = rospy.Rate(5.0)
        self.path_moved = Path()
        self.path = Path()
        self.br = tf.TransformBroadcaster()
        self.pub_path_moved = rospy.Publisher("path_moved", Path, queue_size= 10)
        self.pub_path =rospy.Publisher("path", Path, queue_size = 10)
        self.move_publish = rospy.Publisher("cmd_vel", Twist, queue_size = 10)
        self.calculate_path()
        self.spin()

    def calculate_path(self):
        R = self.wheel_base / math.tan(self.theta) # calculate radius of circled moving by theta. 
        obstacles = rospy.get_param("object_position") #get param obstacles
        if obstacles[0][1] > obstacles[1][1]:
            pt = obstacles[0]; ps = obstacles[1]
        else :
            pt = obstacles[1]; ps = obstacles[0]

        #calculate x_ss, y_ss
        xt = pt[0]; yt = pt[1]; xs = ps[0]; ys =ps[1] #vector direction 
        x,y  = symbols("x y")

        a = yt - ys; b = -(xt-xs); c = -(yt-ys) * xt + (xt-xs) * yt
        eq1 = ((xt -xs) * (x - xt) + (yt - ys)*(y - yt))
        eq2 = ((a *x + b*y + c )/(np.sqrt(a ** 2 + b ** 2)) - 1)

        sol_dict = solve((eq1, eq2), (x ,y))
        x_move = sol_dict[x]; y_move = sol_dict[y]; self.move_point = [x_move, y_move]
        xr = x_move - R; yr = y_move 
        
        x_cen = ( x_move + xs) / 2
        alpha_cen = -math.acos((x_cen - xr)/ R)

        y_cen = R * math.sin(alpha_cen) + yr
        x_circle2 = 2 * x_cen - xr; y_circle2 = 2 * y_cen - yr
        
        
        alpha = np.linspace(0 , alpha_cen, 5)
        
        for index, num_alpha in enumerate(alpha):
            if index == (len(alpha) - 1):
                break         
            self.path.header.frame_id ="path"
            self.path.header.stamp = rospy.Time.now()
            posed = PoseStamped()
            x_1 = R * np.cos(num_alpha) + xr; y_1 = R * np.sin(num_alpha) + yr
            x_2 = R * np.cos(num_alpha  + 1) + xr; y_2 = R * np.sin(num_alpha + 1) + yr 

            posed.pose.position.x = x_1
            posed.pose.position.y = y_1
            angle = tf.transformations.quaternion_from_euler(0, 0 , math.atan2((y_1 - y_2) , (x_1 - x_2 )))
            posed.pose.orientation.z = angle[2]
            posed.pose.orientation.w = angle[3]
            self.path.poses.append(posed)
        
        x_turn = self.path.poses[-1].pose.position.x
        y_turn = self.path.poses[-1].pose.position.y
        angle_turn = tf.transformations.euler_from_quaternion([0, 0, self.path.poses[-1].pose.orientation.z, self.path.poses[-1].pose.orientation.w])
        print(x_turn, y_turn, angle_turn)
        self.center_point = [x_turn, y_turn, angle_turn ]

        x_goal = xt
        alpha2 = math.acos((x_goal - x_circle2)/R)
        y_goal = R * math.sin(alpha2) + y_circle2
        
        alpha2 = np.linspace(-alpha_cen, alpha2, 5)

        for index2, num_alpha2 in enumerate(alpha2):
            if index2 == (len(alpha2) - 1):
                break
            self.path.header.frame_id ="path"
            self.path.header.stamp = rospy.Time.now()

            posed = PoseStamped()
            x_1 = R * np.cos(num_alpha2) + x_circle2; y_1 = R * np.sin(num_alpha2) + y_circle2
            x_2 = R * np.cos(num_alpha2  + 1) + x_circle2; y_2 = R * np.sin(num_alpha2 + 1) + y_circle2
            posed.pose.position.x = x_1
            posed.pose.position.y = y_1
            angle2 = tf.transformations.quaternion_from_euler(0, 0 , math.atan2((y_2 - y_1) , (x_2 - x_1 )))
            posed.pose.orientation.z = angle2[2]
            posed.pose.orientation.w = angle2[3]
            self.path.poses.append(posed)

        self.br.sendTransform((0.0, 0.0, 0.0),(0.0,0.0,0.0,1.0),
                                rospy.Time.now(),"path", "map")  

        x_turn = self.path.poses[-1].pose.position.x
        y_turn = self.path.poses[-1].pose.position.y
        angle_turn = tf.transformations.euler_from_quaternion([0, 0, self.path.poses[-1].pose.orientation.z, self.path.poses[-1].pose.orientation.w])
        print(x_turn, y_turn, angle_turn)
        self.goal_point = [x_turn, y_turn, angle_turn ]
    def spin(self):
        time_begin = time.time()
        index_num = 3
        while not rospy.is_shutdown():
            self.pub_path.publish(self.path)
            odom = take_odomemetry() #[x, y , angular]
            self.rate.sleep()

    def calculate_velocity():
        K_v = 0.4
        K_p = 0.35

        x_init = 0
        y_init = 0

        x_goal = 1
        y_goal = 1

        theta_init = 0.0
        theta_goal = -math.pi / 2
        v =  -K_v* math.sqrt((x_init - x_goal) ** 2 + (y_init - y_goal) ** 2)
        theta = math.atan2( (y_goal - y_init),(x_goal - x_init))
        phi = K_p * (theta_goal - theta_init)
        print(distance)
        return v, phi, distance 

if __name__ == "__main__":
    parking_ob = move_parking()
