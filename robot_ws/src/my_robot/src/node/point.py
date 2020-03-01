#!/usr/bin/env python

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


class Cal_point(): #Tính toán điểm sẽ đi từ tọa độ vật cản
  #Đầu vào point_obs là điểm vật cản, mode chọn chế độ slalom hoặc parking.
  #function mode_parking tính toán điểm nằm giữa 2 vật cản
  #function mode_slalom tính toán các điểm theo quỹ đạo

  def __init__(self, mode, point_obs):
    self.mode = mode
    self.point_seq_obs = point_obs
    self.point_move = []
    if self.mode == "slalom":
      self.mode_slolam()
      self.point_move = [[self.point_move[x], self.point_move[x + 1], self.point_move[x + 2]] for x in range(0, len(self.point_move), 3)]

    elif self.mode == "parking":
      self.mode_parking()
    else:
      Print("Don't have action !")

  def mode_parking(self):
    p1 = self.point_seq_obs[0]; p2 = self.point_seq_obs[1]
    x = float(p1[0] + p2[0]) / 2 ; y = float(p1[1]  + p2[1]) / 2
    a = float(p2[0] - p1[0]); b = float(p2[1] - p1[1]);
    angle = angle1 = math.degrees(math.acos((a)/ (np.sqrt(a ** 2 + b **2 ))))
    self.point_move.append([x,y, angle])

  def mode_slolam(self):
    biendoi = -1
    for index in range(len(self.point_seq_obs)):
      if index  + 1== len(self.point_seq_obs):
        break 
      p1 = self.point_seq_obs[index]; 
      p2 = self.point_seq_obs[index +  1]

      a = float(p2[0] - p1[0]); b = float(p2[1] - p1[1]);
      x , y = symbols('x y')

      eq1 = Eq(a * (x - p1[0]) + b * (y - p1[1]))
      c = - (-b * p1[0] + a * p1[1])
      eq2 = Eq(biendoi *  (-b * x + a * y + c)/(math.sqrt(a ** 2 + b ** 2)) - 2 )

      sol_dict = solve((eq1, eq2), (x ,y))
      x = sol_dict[x]; y = sol_dict[y]; 
      x2 = float(p1[0] + p2[0]) / 2; y2 = float(p1[1] + p2[1]) / 2;
      x3 = x2 * 2 - x; y3 = y2 * 2 -y ; 

      angle1 = math.degrees(math.acos((a)/ (np.sqrt(a ** 2 + b **2 ))))
      
      if biendoi == - 1:
        angle2 = angle1 + 90.0
        angle3 = angle2 - 90.0
      else:
        angle2 = angle1 - 90.0
        angle3 = angle2 + 90.0

      if index == 0 :
        self.point_move.extend([x, y, angle1] + [ x2, y2, angle2] +  [x3, y3, angle3])
      else:
        self.point_move.extend([x2, y2, angle2] +  [x3, y3, angle3] )

      biendoi = -biendoi

class Move_seq(): #Gửi nhiều điểm tọa độ bằng action lên Rviz
  def __init__(self, point_seq):
    rospy.init_node("move_base_sequence")
    self.point_seq = point_seq
    self.goal_cnt = 0
    self.pose_seq = []
    for point in self.point_seq:
      quater = quaternion_from_euler(0, 0, point[2] * math.pi/ 180)
      point_g = Pose()
      point_g.position.x = point[0]; point_g.position.y = point[1]
      point_g.orientation.x = quater[0]; point_g.orientation.y = quater[1]; point_g.orientation.z = quater[2]; point_g.orientation.w = quater[3];
      self.pose_seq.append(point_g) 

    self.client = actionlib.SimpleActionClient("move_base",MoveBaseAction)
    wait = self.client.wait_for_server(rospy.Duration(5.0))  
    if not wait:
      rospy.logerr("Action server not available !")
      rospy.signal_shutdown("Action server not available !")
      return 
    self.publish_goal_action()
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

if __name__ == "__main__":
  p_obs = [[3.01, -2.89], [0.5, -2.9], [-2.2, -3.06], [-4.8, -3.08]] # Gửi tọa độ vật cản 
  def_point = Cal_point("slalom", p_obs ) 
  point_move = def_point.point_move
  print(point_move)
  Move_seq(point_move)
  #result = publish_goal_action()
