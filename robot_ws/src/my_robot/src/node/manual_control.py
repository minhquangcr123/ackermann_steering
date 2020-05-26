#!/usr/bin/env python
import roslib
import numpy as np
import rospy
import math
import threading
from std_msgs.msg import Float64
from controller_manager_msgs.srv import ListControllers
from geometry_msgs.msg import Twist
roslib.load_manifest('my_robot')
import tf
from nav_msgs.msg import Odometry
from std_msgs.msg import Header
from gazebo_msgs.srv import GetModelState, GetModelStateRequest

class Ackermann(object):

    def __init__(self):
        #Init node 
        rospy.init_node("ackermann_controller", anonymous = True   )
        list_ctr = rospy.ServiceProxy("/controller_manager/list_controllers", ListControllers)
        list_ctr.wait_for_service()
        respone = list_ctr()
        #Set defaut parameters
        self.D_dia = 1.0
        self.time_out = 1.0
        self.frequency = rospy.Rate(30.0)
        self._cmd_timeout = 60* 10

        #get parameters 
        (self.left_steering_name, self.left_steering_ctrlr_name, 
        self.left_front_axle_ctrlr_name, self.left_front_iv_cir, self.left_rear_link_name, self.left_rear_axle_ctrlr_name, self.left_rear_iv_cir) = self.get_wheel_param("left") 


        (self.right_steering_name, self.right_steering_ctrlr_name, 
        self.right_front_axle_ctrlr_name, self.right_front_iv_cir, self.right_rear_link_name, self.right_rear_axle_ctrlr_name, self.right_rear_iv_cir) = self.get_wheel_param("right") 


        self.shock_param = rospy.get_param("shock_absorbers",[])
        #publish shock absorbers
        try:
            for shock_absor in self.shock_param:
                ctrlr_name = shock_absor["controller_name"] 
                equi = shock_absor["equilibrium_position"]
                pub = rospy.Publisher(ctrlr_name +"/command", Float64, latch = True, queue_size = 1) 
                self.wait_controller(respone, ctrlr_name)
                pub.publish(equi)
        except:
            rospy.logwarn("Failed in some where in block try, test again please !")
        #parameter to control 
        self._last_cmd_time = rospy.get_time()

        self._ackermann_cmd_lock = threading.Lock()
        self._steer_ang = 0.0# Steering angle 
        self._steer_ang_vel = 0.0# Steering angle velocity
        self._speed = 0.0
        self._accel = 0.0     # Acceleration

        self._last_steer_ang = 0.0 # Last steering angle
        self._theta_left = 0.0      # Left steering joint angle
        self._theta_right = 0.0     # Right steering joint angle

        self._last_speed = 0.0
        self._last_accel_limit = 0.0  # Last acceleration limit
        # Axle angular velocities
        self._left_front_ang_vel = 0.0
        self._right_front_ang_vel = 0.0
        self._left_rear_ang_vel = 0.0
        self._right_rear_ang_vel = 0.0

        #Tread and wheel base
        cor_left_steer = self.get_coordinate(self.left_steering_name)
        cor_right_steer = self.get_coordinate(self.right_steering_name)
        self.dis_stee_div2 = np.linalg.norm(cor_left_steer - cor_right_steer) / 2
        lrw_coor = self.get_coordinate(self.left_rear_link_name)
        rrw_coor = np.array([0.0] * 3) #note : in other source used 0 coordinate
        front_center = (cor_left_steer + cor_right_steer) / 2
        rear_center = (lrw_coor + rrw_coor) / 2
        self.wheel_base = np.linalg.norm(front_center - rear_center)

        self.inv_wheel_base =  1 / self.wheel_base
        self.square_wheel_base = self.wheel_base ** 2

        # publishers 
        self.left_steer_pub = self.creat_cmd_pub(respone, self.left_steering_ctrlr_name)
        self.right_steer_pub = self.creat_cmd_pub(respone, self.right_steering_ctrlr_name)

        self.left_front_axle_pub = self.creat_cmd_pub(respone, self.left_front_axle_ctrlr_name)
        self.right_front_axle_pub = self.creat_cmd_pub(respone, self.right_front_axle_ctrlr_name)
        self.left_rear_axle_pub = self.creat_cmd_pub(respone, self.left_rear_axle_ctrlr_name)
        self.right_rear_axle_pub = self.creat_cmd_pub(respone, self.right_rear_axle_ctrlr_name)

    def callback(self, data):
        self._steer_ang = data.angular.z / 1 * (math.pi/4)
        self._speed = data.linear.x  * 10
        print(self._steer_ang)

    def spin(self) :
        #rospy.Subscriber("/cmd_vel", Twist, self.callback)
        rospy.Subscriber("/cmd_vel_mux/input/teleop", Twist, self.callback)
        last_time = rospy.get_time()
        while not rospy.is_shutdown():
            t = rospy.get_time()
            delta_t = t - last_time
            last_time = t
            if (self._cmd_timeout > 0.0 and t - self._last_cmd_time > self._cmd_timeout):
                steer_angel_chaned, center_y = self.control_steer(self._last_steer_ang, 0.0, 0.001)
                self.control_axle(0.0, 0.0, 0.0, steer_ang_changed, center_y,self._steer_ang  )
                
            elif delta_t >0.0:
                with self._ackermann_cmd_lock:
                    steer_ang = self._steer_ang  #0
                    steer_ang_vel = self._steer_ang_vel #0
                    speed = self._speed #0
                    accel = self._accel #0
                steer_ang_changed, center_y = self.control_steer(steer_ang, steer_ang_vel, delta_t)
                self.control_axle(speed, accel, delta_t, steer_ang_changed,
                                 center_y, steer_ang)
            


            self.left_steer_pub.publish(self._theta_left)
            self.right_steer_pub.publish(self._theta_right)
            
            self.left_front_axle_pub.publish(self._left_front_ang_vel)
            self.right_front_axle_pub.publish(self._right_front_ang_vel)
            self.left_rear_axle_pub.publish(self._left_rear_ang_vel)
            self.right_rear_axle_pub.publish(self._right_rear_ang_vel)

            #print(self._left_front_ang_vel,self._right_front_ang_vel,self._left_rear_ang_vel,self._right_rear_ang_vel)
            self.tranform()

            self.frequency.sleep()
    def tranform(self):
        odom_pub = rospy.Publisher("/my_odom", Odometry, queue_size = 10)
        odom_broadcaster = tf.TransformBroadcaster()

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
        #odom_quat = tf.transformations.quaternion_from_euler(0, 0, odom.pose.pose.orientation.z)
        odom_broadcaster.sendTransform((odom.pose.pose.position.x, odom.pose.pose.position.y, 0), 
                                            (0, 0, quat[2], quat[3]), rospy.Time.now(), "base_link","odom")
        odom_pub.publish(odom)
        
    def control_axle(self, speed, accel_limit, delta_t, steer_angle_changed, center_y, angle):
        #control axle
        if accel_limit >0.0:
            self._last_accel_limit = accel_limit
            accel = (speed - self._last_speed) / delta_t #could be negative or positive
            accel = max(-accel_limit,min(accel, accel_limit))
            veh_speed = self._last_speed + delta_t * accel
        else:
            self._last_accel_limit = accel_limit
            veh_speed = speed
        if veh_speed != self._last_speed or steer_angle_changed:
            #print(center_y)
            if angle < -0.00001 or angle > 0.00001:
                self._last_speed = veh_speed
                left_dist = center_y - self.dis_stee_div2
                right_dist = center_y + self.dis_stee_div2
                #print(left_dist, right_dist)

                    #Front
                #gain = (1 * math.pi) * veh_speed / abs(center_y)
                gain = 6 *  veh_speed / abs(center_y)
                r_in = np.linalg.norm(left_dist ** 2 + self.square_wheel_base)
                self._left_front_ang_vel = gain * r_in* self.left_front_iv_cir # v_angle = K * v_linear * omega = v_linear * 2 pi / T
                
                r_out = np.linalg.norm(right_dist ** 2 + self.square_wheel_base)
                self._right_front_ang_vel = gain * r_out * self.right_front_iv_cir
                
               
                    #Rear
 
                self._left_rear_ang_vel = gain * abs(left_dist) * self.left_front_iv_cir
                self._right_rear_ang_vel = gain * abs(right_dist) * self.left_front_iv_cir
               
            else :
                self._last_speed = veh_speed
                left_dist = center_y - self.dis_stee_div2
                right_dist = center_y + self.dis_stee_div2

                gain = (2 * math.pi) * veh_speed / abs(center_y)
                self._left_rear_ang_vel = gain * left_dist * self.left_front_iv_cir
                self._right_rear_ang_vel = gain * right_dist * self.left_front_iv_cir
                self._left_front_ang_vel = gain * left_dist * self.left_front_iv_cir
                self._right_front_ang_vel = gain * right_dist * self.left_front_iv_cir
    
    def control_steer(self, steer_ang, steer_ang_vel_limit, delta_t):
        #control steering
        if steer_ang_vel_limit >0.0:
            angle_vel = (steer_ang - self._last_steer_ang) / delta_t
            angle_vel = max(-steer_ang_vel_limit, min(angle_vel, steer_ang_vel_limit))
            theta = self._last_steer_ang + angle_vel * delta_t
        else :
            theta = steer_ang #limit control angle theta
        center_y = self.wheel_base * math.tan(math.pi/2 - theta)
        steer_anl_changed = theta != self._last_steer_ang
        if steer_anl_changed :
            self._last_steer_ang = theta
            self._theta_left =  self._get_steer_ang(math.atan(self.inv_wheel_base * (center_y - self.dis_stee_div2)))
            self._theta_right = self._get_steer_ang(math.atan(self.inv_wheel_base * (center_y + self.dis_stee_div2)))
        return  steer_anl_changed, center_y

    def creat_cmd_pub(self, list_ctrlr, ctrlr_name):
        #creat cmd publisher
        self.wait_controller(list_ctrlr, ctrlr_name)
        return rospy.Publisher(ctrlr_name +"/command", Float64, queue_size = 1)
        
    def get_coordinate(self, link):
        listener = tf.TransformListener()
        rate = rospy.Rate(10.0)
        while not rospy.is_shutdown():
            try:
                (trans,rot) = listener.lookupTransform(self.right_rear_link_name, link, rospy.Time(0))
                #print(trans)
                return np.array(trans)
            except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
                pass
            rate.sleep()
        
    def wait_controller(self, respone_list_ctrlr, ctrlr_name):
        #wait for the specified controller to be in "running" state. Command 
        # can be lost if they are published before their controller is running.
        for ctrlr in respone_list_ctrlr.controller:
            if ctrlr.name == ctrlr_name:
                if ctrlr.state == "running":
                    return
                rospy.sleep(0.1)
                break
    def get_wheel_param(self, side):
        #front wheel
        prefix_front = "/" + side + "_front_wheel/"
        front_steering_name = rospy.get_param(prefix_front + "steering_link_name")
        front_steering_control_name = rospy.get_param(prefix_front +"steering_controller_name")
        front_axle_ctrlr_name = rospy.get_param(prefix_front +"axle_controller_name", None)
        front_iv_cir = self.get_circumference(prefix_front)
 
        
        #rear wheel
        prefix_rear = "/" + side +"_rear_wheel/"
        rear_axle_name = rospy.get_param(prefix_rear +"link_name")
        rear_axle_ctrlr_name = rospy.get_param(prefix_rear +"axle_controller_name", None)
        rear_iv_cir = self.get_circumference(prefix_rear)

        return front_steering_name, front_steering_control_name, front_axle_ctrlr_name, front_iv_cir,rear_axle_name, rear_axle_ctrlr_name, rear_iv_cir

    def get_circumference(self, prefix):
        dia = float(rospy.get_param(prefix +"diameter",self.D_dia))
        if dia <= 0.0:
            dia = self.D_dia
            rospy.logwarn("The specified wheel diameter is invalid. "
                          "The default diameter will be used instead.")
        return 1/(math.pi * dia)  
    def _get_steer_ang(self, phi):
        # Return the desired steering angle for a front wheel.
        if phi >= 0.0:
            return (math.pi / 2) - phi
        return (-math.pi / 2) - phi

if __name__ == "__main__":
    ack = Ackermann()
    ack.spin()