#!/usr/bin/env python

import numpy as np
import tf
import rospy, math
from geometry_msgs.msg import PoseStamped
import actionlib 
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from geometry_msgs.msg import Pose, Point, Quaternion
from tf.transformations import quaternion_from_euler
from geometry_msgs.msg import Twist
from sympy import symbols, Eq, solve

import rospy
#[[2.702139790531469, -4.299077367436627, 88.4541277477824]]


def get_move_param():
    results = []
    param = rospy.get_param('move_seq_object') # [[x1,y1,ang1], [x2,y2,ang2], ...,[xn,yn,angn]]
    
    print(param)
    #get param and convert string to list
    list_param = param.split("],")
    for element in list_param:
        element = element.replace("[","")
        element = element.replace("]","")
        
        results.append(element)

    results = [[float(ele.split(',')[0]), float(ele.split(',')[1]), float(ele.split(',')[2])] for ele in results]
    return results

class Move_seq():
    def __init__(self, point_move):
        rospy.init_node("move_seq_action")
        self.publish_vel = rospy.Publisher("cmd_vel", Twist, queue_size= 1)
        vel = Twist()
        vel.linear.x =0.0
        vel.angular.z =0.0
        self.publish_vel.publish(vel)
        self.rate = rospy.Rate(10)
        self.goal_cnt = 0
        self.pose_seq = []
        self.point_move = point_move
        print(self.point_move)
        
        for point in self.point_move:
            quater = quaternion_from_euler(0, 0, point[2] * math.pi/ 180)
            point_g = Pose()
            point_g.position.x = point[0]; point_g.position.y = point[1]
            point_g.orientation.x = quater[0]; point_g.orientation.y = quater[1]; point_g.orientation.z = quater[2]; point_g.orientation.w = quater[3];
            self.pose_seq.append(point_g) 

        self.client = actionlib.SimpleActionClient("move_base", MoveBaseAction)
        rospy.loginfo("Waiting for move_base action server...")
        wait = self.client.wait_for_server(rospy.Duration(5.0)) 

        if not wait:
            rospy.logerr("Action server not available ! Failed")
            rospy.signal_shutdown("Action server not available ! Failed")
            return 
        
        self.publish_goal_action()
        
        #self.move_slalom_open_loop()

    def  done_cb(self, status,result):
        self.goal_cnt += 1
        if status == 3:
            if self.goal_cnt < len(self.pose_seq):
                next_goal = MoveBaseGoal()
                next_goal.target_pose.header.frame_id = "map"
                next_goal.target_pose.header.stamp = rospy.Time.now()
                next_goal.target_pose.pose = self.pose_seq[self.goal_cnt]
                rospy.loginfo("Sending goal pose {} to Action server with pose : {}".format(self.goal_cnt, self.pose_seq[self.goal_cnt]))
                self.client.send_goal(next_goal, self.done_cb)
            else:
                rospy.loginfo("Final goal pose reached !")
                #self.move_slalom()
                rospy.signal_shutdown("Final goal pose reached !")
                return 

    def publish_goal_action(self):
        goal = MoveBaseGoal()
        goal.target_pose.header.frame_id = "map"
        goal.target_pose.header.stamp = rospy.Time.now()
        goal.target_pose.pose = self.pose_seq[self.goal_cnt]
        rospy.loginfo("Sending goal pose {} to Action server with pose : {}".format(self.goal_cnt, self.pose_seq[self.goal_cnt]))
        self.client.send_goal(goal, self.done_cb)
        rospy.spin()
    
    def stop_robot(self):
        vel = Twist()
        vel.linear.x = 0
        vel.linear.z = 0
        return vel

""" def move_slalom_open_loop(self):
        count = self.num_object
        time_move = 1.7
        first_time = rospy.get_time()
        vel = Twist()
        #time_move = [first_time + 0.8* i  for i in range(1,count)] 
        while not rospy.is_shutdown():
            time = rospy.get_time()
            if (time-first_time) > 0.7 + 1.4 + (count - 1) * time_move:
                self.publish_vel.publish(self.stop_robot())
                return
            else:
                print(time)
                if time < first_time + 0.7 :
                    vel.linear.x = 0.24
                    vel.angular.z = 0.7
                    self.publish_vel.publish(vel)
                elif time > first_time + 0.7 and time < first_time + 0.7 + 1.4:
                    vel.linear.x = 0.24
                    vel.angular.z = -0.7
                    self.publish_vel.publish(vel)

                biendoi = 1
                for index_t in range(count):
                    if time > first_time + 0.7 + 1.4 + index_t * time_move and time < first_time + 0.7 + 1.4 + (index_t + 1) * time_move:
                        vel.linear.x = 0.24
                        vel.angular.z = biendoi * 0.7
                        self.publish_vel.publish(vel)
                    biendoi = -biendoi
                    #elif time > first_time + 0.7 + 1.4 + 1.8 and time < first_time + 0.7 + 1.4 + 1.8 + 1.8:
                        #vel.linear.x = 0.25
                        #vel.angular.z = -0.7
                       #self.publish_vel.publish(vel)                    
            
            self.rate.sleep()
            

def move_slalom():
    rospy.init_node("move_seq_action")
    publish_vel = rospy.Publisher("cmd_vel", Twist, queue_size= 1)
    while not rospy.is_shutdown():
        vel = Twist()
        vel.linear.x = 0.1
        vel.angular.z = 0.4
        publish_vel.publish(vel)

def calculate_point_parking(object_obstacles):
    print(object_obstacles)
    if object_obstacles[0][1] > object_obstacles[1][1]:
        p0 = object_obstacles[0]; p1 = object_obstacles[1] 
    else:
        p0 = object_obstacles[1]; p1 = object_obstacles[0] 
        
    xt = p0[0]; yt = p0[1]; xs = p1[0]; ys =p1[1] #vector direction 
    x,y  = symbols("x y")

    a = yt - ys; b = -(xt-xs); c = -(yt-ys) * xt + (xt-xs) * yt
    eq1 = ((xt -xs) * (x - xt) + (yt - ys)*(y - yt))
    eq2 = ((a *x + b*y + c )/(np.sqrt(a ** 2 + b ** 2)) - 2)

    sol_dict = solve((eq1, eq2), (x ,y))
    x_move = sol_dict[x]; y_move = sol_dict[y]; 
    
    angle_move = math.degrees(math.atan2((yt - ys), (xt -xs)))
    print(angle_move)

    return [[x_move, y_move, angle_move]]"""

if __name__ == "__main__":
    move_seq = get_move_param()
    print(move_seq) # get move slalom from point.py
    Move_seq(move_seq)