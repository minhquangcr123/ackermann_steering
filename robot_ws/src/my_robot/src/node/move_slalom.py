#!/usr/bin/env python
#coding:utf-8
import numpy as np
import tf
import rospy, math
from geometry_msgs.msg import PoseStamped
import actionlib 
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from geometry_msgs.msg import Pose, Point, Quaternion
from tf.transformations import quaternion_from_euler
from geometry_msgs.msg import Twist
import tf
from nav_msgs.msg import Path
from nav_msgs.msg import Odometry
from gazebo_msgs.srv import GetModelState, GetModelStateRequest
from std_msgs.msg import Header
import skfuzzy as fuzzy
from skfuzzy import control as ctrl
import matplotlib.pyplot as plt
import time


def get_move_param():
    results = []
    param = rospy.get_param('move_seq_object') # [[x1,y1,ang1], [x2,y2,ang2], ...,[xn,yn,angn]]
    num_object = rospy.get_param("num_object")
    #print(param)
    list_param = param.split("],")
    for element in list_param:
        element = element.replace("[","")
        element = element.replace("]","")
    
        results.append(element)

    results = [[float(ele.split(',')[0]), float(ele.split(',')[1]), float(ele.split(',')[2])] for ele in results]
    return results, num_object

def function_sin_y(x):
    return math.sin(1.55  * x + (math.pi * 0))

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

def fuzzy_control(input_control):
    #Input and output value of control system
    E_p = ctrl.Antecedent(np.arange(0, 2.1, 0.1), "error_distance")
    E_ang = ctrl.Antecedent(np.arange(-180,210, 30), "error_angular")
    O_v = ctrl.Consequent(np.arange(0,2.1,0.01), "velocity")
    O_omega = ctrl.Consequent(np.arange(-22,24,1), "angular_steering")

    #Generate membership of E_p (0 -> 2 metters)
    names_dis = ["Z" , "S", "M", "B" , "VB"] 
    E_p["Z"] = fuzzy.trapmf(E_p.universe, [-0.4, -0.1, 0.1, 0.4])
    E_p["S"] = fuzzy.trimf(E_p.universe, [0, 0.4, 1])
    E_p["M"] = fuzzy.trimf(E_p.universe, [0.4, 1, 1.6])
    E_p["B"] = fuzzy.trimf(E_p.universe, [1.0, 1.6, 2.0])
    E_p["VB"] = fuzzy.trapmf(E_p.universe, [1.6, 1.9, 2.0, 2.2])
    #E_p.view()

    #Generate membership of E_ang (-180 ->  angulars)
    names_ang = ["NB", "NM", "NS", "Z","PS","PM", "PB"]
    E_ang["NB"] = fuzzy.trapmf(E_ang.universe, [-200, -180, -90, -30])
    E_ang["NM"] = fuzzy.trimf(E_ang.universe, [-90, -30, -7])
    E_ang["NS"] = fuzzy.trimf(E_ang.universe, [-30, -7, 0])
    E_ang["Z"] = fuzzy.trimf(E_ang.universe, [-7, 0 ,7])
    E_ang["PS"] = fuzzy.trimf(E_ang.universe, [ 0 , 7, 30])
    E_ang["PM"] = fuzzy.trimf(E_ang.universe, [7,30,90])
    E_ang["PB"] = fuzzy.trapmf(E_ang.universe, [30, 90, 180, 200])
    #E_ang.view()

    #Generate membership of O_v (0 -> 0.2)
    names_ov = ["Z","S","M","B","VB"]
    O_v["Z"] = fuzzy.trapmf(O_v.universe, [-1,-0.5, 0, 0.5])
    O_v["S"] = fuzzy.trimf(O_v.universe, [0, 0.5, 1])
    O_v["M"] = fuzzy.trimf(O_v.universe, [0.5, 1, 1.5])
    O_v["B"] = fuzzy.trimf(O_v.universe, [1, 1.5, 2])
    O_v["VB"] = fuzzy.trapmf(O_v.universe, [1.5,2, 2.5, 3])
    #O_v.view()

    #Generate membership of O_omega (-45 -> 45)
    names_omega  = ["NB", "NM", "NS", "Z", "PS", "PM", "PB"]
    O_omega["NB"] = fuzzy.trapmf(O_omega.universe, [-26,-24,-18, -12])
    O_omega["NM"] = fuzzy.trimf(O_omega.universe, [-18, -12, -6])
    O_omega["NS"] = fuzzy.trimf(O_omega.universe, [-12, -6, 0])
    O_omega["Z"] = fuzzy.trimf(O_omega.universe, [-6,0, 6])
    O_omega["PS"] = fuzzy.trimf(O_omega.universe, [0,6,12])
    O_omega["PM"] = fuzzy.trimf(O_omega.universe, [6,12,18])
    O_omega["PB"] = fuzzy.trapmf(O_omega.universe, [12, 18, 24, 26])
    #O_omega.view()

    #Rule of fuzzy controller.
    rule0 = ctrl.Rule(E_ang["NB"] & E_p["Z"]
                , (O_omega['PS'] , O_v["S"]), label='rule PS-S')
    rule1 = ctrl.Rule(E_ang["NM"] & E_p["Z"]
                | E_ang["NS"] & E_p["Z"]
                , (O_omega['PB'] , O_v["Z"]), label='rule PB-Z')
    rule2 = ctrl.Rule(E_ang["Z"] & E_p["Z"]
                , (O_omega['Z'] , O_v["Z"]), label='rule Z-Z')
    rule3 = ctrl.Rule(E_ang["PS"] & E_p["Z"]
                | E_ang["PM"] & E_p["Z"] 
                , (O_omega['NB'] , O_v["Z"]), label='rule NB-Z')
    rule4 = ctrl.Rule(E_ang["PB"] & E_p["Z"] 
                , (O_omega['NS'] , O_v["S"]), label='rule NS-S')
    rule5 = ctrl.Rule(E_ang["NB"] & E_p["S"]
                | E_ang["NM"] & E_p["S"] 
                | E_ang["NS"] & E_p["S"] 
                | E_ang["NB"] & E_p["M"] 
                , (O_omega['PB'] , O_v["S"]), label='rule PB-S')
    rule6 = ctrl.Rule(E_ang["Z"] & E_p["S"] 
                , (O_omega['Z'] , O_v["S"]), label='rule Z-S')
    rule7 = ctrl.Rule(E_ang["PS"] & E_p["S"] 
                | E_ang["PM"] & E_p["S"] 
                | E_ang["PB"] & E_p["S"]
                | E_ang["PB"] & E_p["M"] 
                , (O_omega['NB'] , O_v["S"]), label='rule NB-S')
    rule8 = ctrl.Rule(E_ang["NM"] & E_p["M"]
                | E_ang["NB"] & E_p["B"]  
                | E_ang["NB"] & E_p["VB"] 
                , (O_omega['PM'] , O_v["S"]), label='rule PM-S')
    rule9 = ctrl.Rule(E_ang["NS"] & E_p["M"] 
                | E_ang["NM"] & E_p["B"] 
                , (O_omega['PS'] , O_v["M"]), label='rule PS-M')
    rule10 = ctrl.Rule(E_ang["Z"] & E_p["M"] 
                , (O_omega['Z'] , O_v["M"]), label='rule Z-M')
    rule11 = ctrl.Rule(E_ang["PS"] & E_p["M"] 
                | E_ang["PM"] & E_p["B"] 
                , (O_omega['NS'] , O_v["M"]), label='rule NS-M')
    rule12 = ctrl.Rule(E_ang["PM"] & E_p["M"] 
                | E_ang["PB"] & E_p["B"] 
                | E_ang["PB"] & E_p["VB"] 
                , (O_omega['NM'] , O_v["S"]), label='rule NM-S')
    rule13 = ctrl.Rule(E_ang["NS"] & E_p["B"] 
                | E_ang["NS"] & E_p["VB"] 
                , (O_omega['PS'] , O_v["B"]), label='rule PS-B')
    rule14 = ctrl.Rule(E_ang["Z"] & E_p["B"] 
                , (O_omega['Z'] , O_v["B"]), label='rule Z-B')
    rule15 = ctrl.Rule(E_ang["PS"] & E_p["B"] 
                | E_ang["PS"] & E_p["VB"] 
                , (O_omega['NS'] , O_v["B"]), label='rule NS-B')
    rule16 = ctrl.Rule(E_ang["NM"] & E_p["VB"] 
                , (O_omega['PM'] , O_v["M"]), label='rule PM-M')
    rule17 = ctrl.Rule(E_ang["Z"] & E_p["VB"] 
                , (O_omega['Z'] , O_v["VB"]), label='rule Z-VB')
    rule18 = ctrl.Rule(E_ang["PM"] & E_p["VB"] 
                , (O_omega['NM'] , O_v["M"]), label='rule NM-M')

    system = ctrl.ControlSystem(rules=[rule0, rule1, rule2, rule3, rule4, rule5
                        , rule6, rule7, rule8, rule9, rule10, rule11, rule12
                        , rule13, rule14, rule15, rule16, rule17, rule18])
    system_sim = ctrl.ControlSystemSimulation(system)

    system_sim.input["error_distance"] = input_control[0]
    system_sim.input["error_angular"] = input_control[1]
    system_sim.compute()
    return system_sim.output
    #O_omega.view(sim = system_sim)
    

def visualize_path_moved(path_moved ,odom_moved):
    pose_moved = PoseStamped()
    pose_moved.header.frame_id = "moved"
    pose_moved.header.stamp = rospy.Time.now()
    pose_moved.pose.position.x = odom_moved[0]
    pose_moved.pose.position.y = odom_moved[1]
    quaternion = tf.transformations.quaternion_from_euler(0,0,odom_moved[2])
    pose_moved.pose.orientation.z = quaternion[2]
    pose_moved.pose.orientation.w = quaternion[3]

    path_moved.poses.append(pose_moved)
    br.sendTransform((0.0, 0.0, 0.0),(0.0,0.0,0.0,1.0),
                            rospy.Time.now(),"path_moved", "map")
    pub_path_moved.publish(path_moved)

def move_robot(linear_x, angular_z):
    velocity_ = Twist()
    if abs(angular_z) > 0.75:
        if abs(linear_x) > 0.5:
            linear_x = linear_x / linear_x * abs(linear_x)
        velocity_.linear.x = linear_x
        velocity_.angular.z = (angular_z / angular_z) * 0.75
    else:
        if abs(linear_x) > 0.5:
            linear_x = linear_x / linear_x * abs(linear_x)
        velocity_.linear.x = linear_x
        velocity_.angular.z = angular_z
        
    pub_velocity.publish(velocity_)

def calculate_path():
    obstacles = rospy.get_param("object_position")
    p0 = obstacles[0]; p1 = obstacles[-1]

    path = Path()
    path.header.frame_id = "path_test"
    path.header.stamp = rospy.Time.now()
    poses = [ ]
    x = np.linspace(p0[0] + 0.5 , p1[0] - 1.5, 30)
    for index_x in range(len(x)):
        pose = PoseStamped()
        y = function_sin_y(x[index_x])
        pose.header.stamp = rospy.Time.now()
        pose.header.frame_id = "test"
        pose.pose.position.x = x[index_x]
        pose.pose.position.y =  y + p1[1]

        if index_x == len(x) - 1:
            break
        
        
        angle = math.atan2((function_sin_y(x[index_x + 1]) - function_sin_y(x[index_x])), (x[index_x + 1] - x[index_x])
        )
        #print(x[index_x], y, angle)
        quaternion = tf.transformations.quaternion_from_euler(0 , 0, angle)
        pose.pose.orientation.x = quaternion[0] 
        pose.pose.orientation.y = quaternion[1]
        pose.pose.orientation.z = quaternion[2]
        pose.pose.orientation.w = quaternion[3]
        poses.append(pose)
    
    odom = take_odomemetry() #[x,y,angle]
    first_pose = PoseStamped()
    first_pose.header.stamp = rospy.Time.now()
    first_pose.header.frame_id = "test"
    first_pose.pose.position.x = odom[0]
    first_pose.pose.position.y =  odom[1]
    first_pose.pose.orientation.x = 0.0
    first_pose.pose.orientation.y = 0.0
    first_pose.pose.orientation.z = tf.transformations.quaternion_from_euler(0,0,odom[2])[2]
    first_pose.pose.orientation.w = tf.transformations.quaternion_from_euler(0,0,odom[2])[3]
    poses.insert(0, first_pose)

    path.poses = poses
    return path
class PID_control_clas():
    def __init__(self, Kp, Ki, Kd):
        self.sample_time = -1
        self.Kp = Kp
        self.Ki = Ki
        self.Kd = Kd   
        self.error = 0
        self.P_term = 0
        self.I_term = 0
        self.D_term = 0
        
    def update(self, c_time, l_time, error):
        delta_t = c_time - l_time
        print(delta_t, " ---------")
        if delta_t > self.sample_time:
            self.P_term = self.Kp * error
            self.I_term += self.Ki * error * delta_t
            self.D_term = self.Kd * error / delta_t   

        return self.P_term + self.I_term + self.D_term
        
def PID_control(current_t, last_time, Kp, Ki, Kd, error):
    sample_time = -1
    delta_t = current_t - last_time
    if delta_t > sample_time:
        Propotial = Kp * error
        Differential = Kd * error / delta_t
        Integrial = Ki * error * delta_t
        last_time = current_t
        #print(delta_t)
    return Propotial + Differential + Integrial

def calculate_error():
    pass
def plot_data(data): #data [[x,y,a,t], [x_1,y_1,a_1,t_1] ...]
    distance = []
    a_plot = []
    time_plot = []
    angular_velo = []

    for data_element in data :
        distance.append(data_element[0])
        a_plot.append(data_element[1])
        time_plot.append(data_element[2])
        angular_velo.append(data_element[3])
    plt.subplot(3,1,1)
    plt.title("distance")
    plt.plot(time_plot, distance)
    plt.subplot(3,1,2)
    plt.title("angular")
    plt.plot(time_plot, a_plot)
    plt.subplot(3,1,3)
    plt.title("angular_velocity")
    plt.plot(time_plot, angular_velo)
    plt.show()
#Action 
rospy.init_node("visual_path")
br = tf.TransformBroadcaster()
rate = rospy.Rate(100)

#move_points, num_obstacles = get_move_param() #movepoint([[x,y,angle], ..])    num_obstacles : integer
pub_path = rospy.Publisher("path", Path,queue_size= 10)
pub_velocity = rospy.Publisher("cmd_vel", Twist, queue_size= 10)
pub_path_moved = rospy.Publisher("path_moved", Path, queue_size= 10)

#move_seq, object_seq = get_move_param() #param move_seq is started point #movepoint([[x,y,angle], ..]) 
path = calculate_path()
rate.sleep()
#path_moved_visualize
path_moved = Path()
path_moved.header.frame_id = "path_moved"
path_moved.header.stamp = rospy.Time.now()

index_p_m = 1 #index of moving point in path. Initial : 0
time_begin = time.time()
last_t = time_begin
time_move_done = 100.0
time_first_point = 10.0

pid_x = PID_control_clas(1.0, 0., 0.00000) #0.00001
pid_y = PID_control_clas(1.0, 0., 0.00000)
pid_angular = PID_control_clas(1., 0., 0.0)
data = []

def get_coor_dinates():
    name_r = "/left_rear_wheel/link_name" 
    name_l = "/right_rear_wheel/link_name"
    rear_r_par = rospy.get_param(name_r)
    rear_l_par = rospy.get_param(name_l)
    listener = tf.TransformListener()
    listener.waitForTransform("/map",rear_r_par,  rospy.Time(), rospy.Duration(1.0))
    listener.waitForTransform( "/map", rear_l_par,rospy.Time(), rospy.Duration(1.0))
    trans1,rot = listener.lookupTransform("/map", rear_r_par, rospy.Time())
    trans2,rot = listener.lookupTransform("/map", rear_l_par, rospy.Time())
    #print(trans)
    return np.array(trans1), np.array(trans2)
            

while not rospy.is_shutdown():
    time_current = time.time()

    pub_path.publish(path) #visualize path for move
    br.sendTransform((0.0, 0.0, 0.0),(0.0,0.0,0.0,1.0),
                            rospy.Time.now(),"path_test", "map")
    
    # Calculate error
    time_span = np.linspace(time_begin + time_first_point, time_begin + time_first_point + time_move_done, len(path.poses)) #calculate t_i with each point in path 
    time_span = np.insert(time_span, 0, time_begin)
    #time_span.insert(0, time_begin)
    odom = take_odomemetry() #[x,y,angle]
    visualize_path_moved(path_moved, odom)   #visualize path robot have moved
    odom_r, odom_l = get_coor_dinates() #rear odometry of car
    odom_rear = (odom_r + odom_l)/2
    x = odom_rear[0]; y = odom_rear[1]; angle = odom[2]; #take odometry
    angular_p = [0,0, path.poses[index_p_m].pose.orientation.z, path.poses[index_p_m].pose.orientation.w]
    angular_p = tf.transformations.euler_from_quaternion(angular_p)[-1]
    two_angular = [angular_p, angle]
    #tranlate angular to 0 -> pi ( before translate : angular have values (0 -> +180, 0 -> -180 )  )
    for index,ang in enumerate(two_angular):
        if ang < 0:
            two_angular[index] = 2 * math.pi + ang

    error_angular = two_angular[0] - two_angular[1] #error angular
    error_distance_x = path.poses[index_p_m].pose.position.x - x
    error_distance_y = path.poses[index_p_m].pose.position.y - y
    #print(error_distance_x, error_distance_y)
    error_distance = np.sqrt(error_distance_x ** 2 + error_distance_y ** 2)
    delta_t_desire = time_span[index_p_m] - time_current # time beetween desired moving point and feedback odom 
    
    v_linear = np.sqrt((error_distance_x / delta_t_desire ) ** 2 + (error_distance_y / delta_t_desire) ** 2)
    v_omega = math.atan((error_angular * 0.335004153719)/ (delta_t_desire * v_linear)) # 0.335004153719 lenght car chassis

    
    #calculate velocity for move robot
    if index_p_m == (len(path.poses) - 1 ): # stop robot when reach goal
        move_robot(0.0, 0.0) 
    else:
        output_velocity_x = pid_x.update(time_current, last_t, error_distance_x)
        #output_velocity_x = PID_control(time_current, last_t, 1., 0., 0, error_distance_x)
        output_velocity_y = pid_y.update(time_current, last_t, error_distance_y)
        #output_velocity_y = PID_control(time_current, last_t, 2., 0. , 0, error_distance_y)
        v_linear = np.sqrt((output_velocity_x / delta_t_desire ) ** 2 + (output_velocity_y / delta_t_desire) ** 2)
        output_angular = pid_angular.update(time_current, last_t, error_angular)
        #output_angular = PID_control(time_current, last_t, 2 , 0.0 ,0 ,error_angular)
        v_omega = math.atan((output_angular * 0.335004153719)/ (delta_t_desire * v_linear)) # 0.335004153719 lenght car chassis
        # take data for plot
        data.append([error_distance, error_angular, time_current, v_omega])
        
        print("sai so giua hien tai va thoi gian mong muon:{}".format(delta_t_desire))
        print("diem :{} \nsai so khoang cach :{} \nsai so goc: {} \ntoc do :{}, {}".format(index_p_m, error_distance, math.degrees(error_angular), v_omega, v_linear))
        print("\n")
        if v_linear > 0.6:
            move_robot(0.0,0.0)
            break
        """
        if index_p_m > 1:#
            move_robot(0.0, 0.0)
            break"""
        #if abs(error_distance_x)  < 0.2 and abs(error_distance_y) < 0.2:
        if abs(error_angular)  < 5.0 and error_distance < 0.2 or delta_t_desire < 1.0 :
            move_robot(0.0, 0.0)
            index_p_m +=1
        
        move_robot(v_linear, v_omega)
        #move_robot(output_velocity / delta_t, output_angular/ delta_t)
            
    
    last_t = time_current
    rate.sleep()
print(time_span[1] - time_span[0], time_span[2] - time_span[1])
plot_data(data)      
    #plt.show()
    

