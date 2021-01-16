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

def distance_norm2(p1, p2):
  return np.sqrt((p1[0] - p2[0]) ** 2+ (p1[1] - p2[1])** 2)

class Cal_point(): 
  #Tinh toan diem se di tu toa do vat can
  #Input point_obs la toa do diem cac vat can
  #function mode_parking tinh toan diem nam giua 2 vat can
  #function mode_slalom tinh toan diem theo quy dao slalom

  def __init__(self, mode):
    rospy.init_node("objetct", disable_signals=True)
    self.point_seq_obs = [] # Declare Instance attribute
    self.mode = mode
    self.sub_object = message_filters.Subscriber("raw_obstacles", Obstacles)
    self.sub_laser = message_filters.Subscriber("scan", LaserScan)

    ts = message_filters.TimeSynchronizer([self.sub_object, self.sub_laser], 10)
    self.signal_laser_plot = []

    if self.mode == "slalom" or self.mode == "slalom2" or self.mode == "parking":
      ts.registerCallback(self.callback)
      rospy.spin()
        #print(self.point_seq_obs)
      all_point, a_object = self.return_list_point()
      print(a_object)

      self.point_seq_obs = a_object #[[x,y],[x,y], ...[x,y]]
      self.point_move = []
    
    elif self.mode == "all":
      self.all_mode()
      rospy.set_param("move_seq_object", "{}".format(self.point_move))

      #self.laser_sub = rospy.Subscriber("/scan", LaserScan, self.callback_mode_all)
      #rospy.spin()
      #print(len(self.signal_laser_plot))                                                                                     
      #plt.plot(np.arange(0,len(self.signal_laser_plot[-1])), self.signal_laser_plot[-1])
      #plt.show()
      

    if self.mode == "slalom":
      self.mode_slolam()
      self.point_move = [[self.point_move[x], self.point_move[x + 1], self.point_move[x + 2]] for x in range(0, len(self.point_move), 3)]
      #print(self.point_move)
      # condition for denois and take object point
      rospy.set_param("move_seq_object", "{}".format(self.point_move))
      
    elif self.mode == "parking":
      self.calculate_point_parking(self.point_seq_obs)
      print("Parking mode ! Point for move : ", self.point_move)
      rospy.set_param("move_seq_object", "{}".format(self.point_move))
      rospy.set_param("object_position", self.point_seq_obs)
    
    elif self.mode == "slalom2":
      self.mode_slalom2()
      print(self.point_move, self.point_seq_obs)
      rospy.set_param("move_seq_object", "{}".format(self.point_move))
      rospy.set_param("object_position", self.point_seq_obs)
    else:
      print("Done !")

  def all_mode(self):
    self.point_move = [[8.179545, -0.970493, -1.563102/ math.pi * 180]]
    
  def callback_mode_all(self, data_laser):
    self.signal_laser_plot.append(data_laser.ranges)


  def callback(self,data_obj, data_laser ):
    if data_obj.circles:
      print(data_obj.circles)
      for circle in data_obj.circles:
          self.point_seq_obs.append([circle.center.x, circle.center.y])  



  def mode_slalom2(self):
    p1 = self.point_seq_obs[0]; 
    p2 = self.point_seq_obs[1]
    a = float(p2[0] - p1[0]); b = float(p2[1] - p1[1]);
    x = (p1[0] * 2 - p2[0]); y = (p1[1] * 2 - p2[1]) ;
    angle = math.degrees(math.acos((a)/ (np.sqrt(a ** 2 + b **2 ))))

    self.point_move.append([x,y,angle])

  
  def calculate_point_parking(self, object_obstacles):
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

    self.point_move.append([x_move, y_move, angle_move])
    #return []

  def return_list_point(self):
    index = 0
   
    in_list = self.point_seq_obs
    #in_list = [[2.6514299260283227, -1.9582355587142688], [2.7831134099359676, 5.689925089233745], [2.6013741644828756, -1.9812601513618415], [2.7781811459911476, 5.702822611136763], [2.6186474139280245, -1.9908028753724927], [2.833006268436419, 5.7074659658631255], [2.6231961151485423, -1.9790362385294937], [2.584979957181754, -1.985262872091171], [2.7563003003541273, 5.686154342198654], [2.789379287845498, 5.708546580916547], [2.595296587479631, -1.9977274481650058], [2.788295599717343, 5.6725955772435945], [2.5927860734717263, -2.0109785497621235], [2.8818874066975035, 5.696106015626201], [2.8309080414473398, 5.670955064078793], [2.571817429584394, -2.00802822964284], [2.8150200510576253, 5.67072344451447], [2.8708324021004348, 5.724723156326364], [2.573835516939212, -2.0221405139969573], [2.8372735711659836, 5.695325342302995], [2.5572950174137823, -2.0221080296893374], [2.866947068281691, 5.711250163669706], [2.7746855757498716, 5.667789244125478], [2.5377911951950756, -2.02039917068665], [2.758491056278556, 5.660919444769176], [2.534702241690454, -2.0330415111009774], [2.5209687816329147, -2.0323158248076325], [2.768944635752244, 5.657398752984577], [2.50240359549907, -2.029554495201518], [2.713817376516256, 5.670784838988336], [2.743168323633236, 5.685176150908428], [2.4687672150737736, -2.033881361180096], [2.4854577245574765, -2.035452758703265], [2.460955331818729, -2.0410112581707986], [2.442039257791871, -2.0355162373662323], [5.316476656049152, 13.921635768755399], [2.7259243194648386, 5.6806519472873], [5.435146013350831, 13.913943251254718], [2.8256813319157503, 5.72060577786573], [5.333354938661845, 13.936005803524179], [2.6755366465146007, 5.676223653665345], [2.4199659359426136, -2.0352280057313754], [2.7014675831164565, 5.691175971735417], [2.39953689379755, -2.042028336191903], [2.733820367779677, 5.697312725811294], [2.413011169988133, -2.0517397676777094], [2.7515995691815642, 5.704220031292266], [2.384556852632262, -2.050401652271397], [2.3719112678099092, -2.050004034076651], [2.6972911187557997, 5.677099463110142], [2.352227180635984, -2.051894545368528], [2.7238104382331807, 5.705097611790801], [2.3361792444717517, -2.0446255371894217], [2.323328709285211, -2.043355608490871], [2.6565446967612587, 5.679867409113985], [2.3292205953242444, -2.055496735640861], [2.28804384334209, -2.0356776302212674], [2.7992158987146896, 0.4458734372483386], [2.653139465905724, 5.69580009640964], [2.852411580470647, 0.44516740718927794], [2.6917822758634147, 5.712831738554478], [2.2961823729167863, -2.050844316900853], [2.8786632019856357, 0.4316332427594398], [2.2783782770082355, -2.051581465129379], [2.8567696931231756, 0.43203760700365956], [2.641653930863708, 5.68519363084002], [2.8350197056639757, 0.4383321659490327], [2.6826198798598972, 5.695779401279211], [2.6871125709121046, 5.686285492378852], [2.672227399027194, 5.685958439891348], [2.851212388551164, 0.42335949696352326], [2.7476888367269083, 5.713530370898883], [2.8000976697993982, 0.4150142914961732], [2.817702154122396, 0.42990624559373813], [2.7384345178146274, 5.731375414854032], [2.7920995926194196, 0.42802048233681234], [2.702172511770429, 5.720170421876654], [2.7127256059133207, 5.722666679332971], [2.7748206638477004, 0.42875952054945765], [2.7476154534826875, 0.43645680723993463], [2.7263244676984866, 0.44707829405060684], [2.76981645771164, 0.4187659727806352], [2.725948900654808, 0.42004767362462325], [2.6808327976654267, 0.4496826451568128], [2.7108447370919895, 0.4142321561660365], [2.686248713777077, 0.40758192745832367], [2.692383757253382, 0.42164256351811535], [2.704394130693574, 0.40596627796391793], [2.6977396897939965, 0.3972371570531059], [2.6499815902832426, 0.41839873231645996], [2.6615017470220756, 0.4071503390684472], [2.6512062768248734, 0.3995274680174141], [2.630187221838559, 0.3993820473835168], [-12.918402435673343, -9.495612371870198], [2.6191689769129285, 0.3948234079625268], [2.646235096500761, 0.3900526667440647], [2.604017957255879, 0.39126292115680217], [6.2191321447214065, 1.7340726509480382], [2.586625454751575, 0.4063706586918099], [2.5957670179739925, 0.3998338709477003], [2.564233171421413, 0.4080210266357842], [2.581510531038583, 0.3914830680448542], [2.5375870005559737, 0.41109568016827625], [2.5235986487923827, 0.4137702917148208], [2.545116151118735, 0.40142609948395125], [2.5075733298566987, 0.41512336620063695], [2.515973395655237, 0.4046161772231417], [2.4946654499220755, 0.4080224357384231], [2.4784586107106654, 0.4064543190907913], [2.504712140446581, 0.3928288859966873], [2.468357542041348, 0.4205019037598924], [2.494387596573866, 0.397849048883685], [1.6774858267158734, -1.5485491572046226], [1.675529660867519, -1.5323280676052162], [1.6865997933792065, -1.5431715714768337], [4.446305965074184, -5.74215899985679], [1.677712154410456, -1.5628421832160801], [1.6858760654037726, -1.5712978231264803], [2.4847334462478976, 0.4191245810275823], [1.6979320006137435, -1.5689155177002252], [1.6965281432645676, -1.586337363411059], [1.7043136120580151, -1.6213258544800848], [1.707655114856649, -1.596742543913671], [2.495609085825551, 0.4228123694913827], [1.713171327593237, -1.6329721029415722], [2.5244523208167884, 0.3939498988322212], [2.504592508787784, 0.40303264571885533], [1.7136908133008195, -1.6473736940337655], [1.7186362703198133, -1.6646038019096894], [1.721526860425348, -1.654999784948226], [1.7218439436703326, -1.639137655228259], [1.724803655636955, -1.6786651447632899], [1.7382686765386186, -1.6582912823543503], [1.7374454161554032, -1.6743797346335343], [2.537115911545537, 0.3944404555173311], [1.7327474783166423, -1.64893422805721], [2.5279322779346134, 0.4039940853897348], [1.7034726624274925, -1.648972958952494], [1.7287663068152488, -1.6663911780609284], [1.394343233423603, 3.196552670762126], [2.534757102283986, 0.37366605501422934], [1.7271866063801846, -1.6889380168233536], [1.7308514081993136, -1.7156744122233554], [2.5182673284431294, 0.38570500994171764], [2.507077183973985, 0.38250134583181317], [1.7359604312490426, -1.7016179118726713], [1.7404988590607555, -1.720041273935033], [1.74822895563458, -1.6931573099571984], [1.7556503330002848, -1.7287408792302528], [2.5112657917230026, 0.36529130861232617], [1.7530365538019415, -1.7590995074589593], [2.4990175473058978, 0.36937824798949936], [2.479475222798573, 0.3758631792551069], [1.7519590050671736, -1.73904560001302], [2.488658721093754, 0.36797268489489515], [2.456003889913349, 0.3906218810596108], [1.7459318935526218, -1.7506099616819943], [2.486531836141874, 0.3562494737860226], [2.4714654076525067, 0.36779177121925116], [2.4509539129011637, 0.374392177665515], [1.7455198982901017, -1.7313112713135979], [2.4610973988485862, 0.3744963690585831], [1.7595483345209348, -1.7487252979645724], [7.337080142408009, 13.924612753498746], [2.5000080078235722, 0.3580495319620464], [7.374141661975393, 13.897355193870297], [1.7635487425422443, -1.7225055238007758], [7.249472379187234, 13.973768439270684], [1.4238384491941147, 3.1750374632417486], [1.755630228955026, -1.706208335431008], [7.282634808154404, 13.93749242445017], [1.4404833667550148, 3.1653421277920497], [7.361945400611802, 13.889561655242321], [1.7181227779251196, -1.7052593159469231], [7.428073470261214, 13.854562488120546], [7.259447761694533, 13.703667572919613], [7.3359123314958605, 13.894403661313866], [2.509660877872089, 0.34930330244347585], [7.326057717135205, 13.908843021651894], [7.304736781404191, 13.902074569980018]]

    out_list = [in_list[0]]
    while len(in_list) > 0 : 
      print(distance_norm2(out_list[index], in_list[0] ))
      if distance_norm2(out_list[index], in_list[0] )< 0.5:
        in_list = [i for i in in_list if distance_norm2(i, out_list[index]) > 0.5]
      else:
        out_list.append(in_list[0])
        in_list = [i for i in in_list if distance_norm2(i, out_list[index]) > 0.5]
        index +=1

    if len(out_list) > 1: # Nornally of condition when strangely detection existed.
      index_num = []
      for  index in range(len(out_list)- 1):
        point_var_1 = out_list[index]
        for index_2 in range(1, len(out_list)):
          point_var_2 = out_list[index_2]
          distance = distance_norm2(point_var_1, point_var_2)
          print(index_2, distance)
          if distance >1.8 and distance < 3.5:
            index_num.append(index); index_num.append(index_2)

      print(out_list, index_num)

    final_list = [out_list[idex_] for idex_ in list(set(index_num))]
      

    return out_list, final_list

 

  """def callback(self,data ):
    rospy.wait_for_service("/gazebo/get_model_state")
    get_model_respone = rospy.ServiceProxy("gazebo/get_model_state", GetModelState)

    for index,segment in enumerate(data.segments):

      p1x = segment.first_point.x ; p1y = segment.first_point.y
      p2x = segment.last_point.x ; p2y = segment.last_point.y
      distance = np.sqrt((p1x - p2x) ** 2 + (p1y - p2y) ** 2)
      #print(distance, segment)
      if distance > 0.9 and distance < 1.0 and len(self.point_seq_obs) < 1:
        self.point_seq_obs.append([p1x, p1y])
        model = GetModelStateRequest()
        model.model_name = "ackerman"
        result = get_model_respone(model)
        pose = result.pose
        #print(pose)
      elif distance > 0.9 and distance < 1.0 and len(self.point_seq_obs) >= 1:
        for point in self.point_seq_obs:
          if abs(point[0] - p1x) > 0.01 and abs(point[1] - p1y) >  0.1:

            model = GetModelStateRequest()
            model.model_name = "ackerman"
            result = get_model_respone(model)
            pose = result.pose
            #print(pose)
            self.point_seq_obs.append([p1x,p1y])
            break"""
  
  
  def mode_modern(self):
    pass

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


  

if __name__ == "__main__":
  #p_obs = [[0.9,7.7],[-2.47, 7.85]] #Gui toa do vat can
  
  print(sys.argv)
  def_point = Cal_point(sys.argv[1]) 

  #result = publish_goal_action()
