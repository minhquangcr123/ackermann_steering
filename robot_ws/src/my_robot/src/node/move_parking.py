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
        self.rate = rospy.Rate(100)
        self.path_moved = Path()
        self.path = Path()
        self.br = tf.TransformBroadcaster()
        self.pub_path_moved = rospy.Publisher("path_moved", Path, queue_size= 10)
        self.pub_path =rospy.Publisher("path", Path, queue_size = 10)
        self.move_publish = rospy.Publisher("cmd_vel", Twist, queue_size = 10)
        self.calculate_path()
        self.spin()

    def visualize_moved(self, odometry):
        self.path_moved.header.frame_id = "path_moved"
        self.path_moved.header.stamp = rospy.Time.now()
        pose_moved = PoseStamped()
        pose_moved.header.frame_id = "point_moved"
        pose_moved.header.stamp = rospy.Time.now()
        pose_moved.pose.position.x = odometry[0]
        pose_moved.pose.position.y = odometry[1]

        quaternion = tf.transformations.quaternion_from_euler(0,0,odometry[2])
        pose_moved.pose.orientation.z = quaternion[2]
        pose_moved.pose.orientation.w = quaternion[3]

        self.path_moved.poses.append(pose_moved)
        self.br.sendTransform((0.0, 0.0, 0.0),(0.0,0.0,0.0,1.0),
                                rospy.Time.now(),"path_moved", "map")
        self.pub_path_moved.publish(self.path_moved)

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
        eq2 = ((a *x + b*y + c )/(np.sqrt(a ** 2 + b ** 2)) - 2)

        sol_dict = solve((eq1, eq2), (x ,y))
        x_move = sol_dict[x]; y_move = sol_dict[y]; 

        #
        xr = x_move - R; yr = y_move 
        
        x_cen = ( x_move + xs) / 2;
        alpha_cen = -math.acos((x_cen - xr)/ R)

        y_cen = R * math.sin(alpha_cen) + yr
        x_circle2 = 2 * x_cen - xr; y_circle2 = 2 * y_cen - yr;
        
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

        x_goal = xt; alpha2 = math.acos((x_goal - x_circle2)/R)
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
    def move_robot(self, linear_x, angular_z):
       
        velocity_ = Twist()
        if abs(linear_x) > 0.1:
            velocity_.linear.x = (linear_x / linear_x) * abs(linear_x)
            velocity_.angular.z = angular_z
        else:
            if angular_z > 0.52:
                velocity_.linear.x = linear_x
                velocity_.angular.z = 0.52
            else:
                velocity_.linear.x = linear_x
                velocity_.angular.z = angular_z
        
        self.move_publish.publish(velocity_)

    def PID_control(self, time_st , Kp, Ki, Kd, error):
        sample_time = 0
        current_time  = time.time() # unix time
        delta_t = current_time - time_st
        if delta_t > sample_time:
            Propotial = Kp * error
            Differential = Kd * error / delta_t
            Integrial = Ki * error * delta_t
        return Propotial + Differential + Integrial

    def spin(self):
        time_begin = time.time()
        index_num = 3
        while not rospy.is_shutdown():
            self.pub_path.publish(self.path)
            odom = take_odomemetry() #[x, y , angular]
            self.visualize_moved(odom)

            # move_robot 
            if index_num == (len(self.path.poses) - 1 ):
                move_robot(0.0, 0.0)
            else:
                angular_car = odom[2]
                angular_p = tf.transformations.euler_from_quaternion([0,0, self.path.poses[index_num].pose.orientation.z, self.path.poses[index_num].pose.orientation.w])[-1]
                if angular_p < 0 :
                    angular_p = math.pi + (math.pi - abs(angular_p)) # 0 - 360)
                if angular_car < 0:
                    angular_car = math.pi + (math.pi - abs(angular_car))

                distance = np.sqrt(float(self.path.poses[index_num].pose.position.x - odom[0]) ** 2
                    + float(self.path.poses[index_num].pose.position.y - odom[1]) ** 2)
                if distance < 0.3 : #or error_angular < 5.0
                    index_num+=1
                
                error_angular = angular_car - angular_p 
                output_control_signal = self.PID_control(time_begin, 0.25,0, 0 , error_angular )
                self.move_robot(self.velocity_linear, -output_control_signal)
                print(index_num, distance, angular_car, angular_p, error_angular, output_control_signal)
            self.rate.sleep()

if __name__ == "__main__":
    parking_ob = move_parking()