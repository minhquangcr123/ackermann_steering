#!/usr/bin/env python 
import _locale
_locale._getdefaultlocale = (lambda *args: ['en_US', 'utf8'])

import matplotlib.pyplot as plt
import numpy as np
from geometry_msgs.msg import Twist
import rospy
import matplotlib.animation as animation


fig = plt.figure()
#creating a subplot 
ax1 = fig.add_subplot(1,1,1)

def animate(i):
    data = open('/home/tran/github/bag_file/file_text.txt','r').read()
    lines = data.split('\n')
    xs = []
    ys = []
    time = 0
    for line in lines[: -1]:
        x = line.split(', ')[0]
        angular = line.split(', ')[1]# Delimiter is comma    
        ys.append(float(x))
        xs.append(float(time))
        time += 1
    
    ax1.clear()
    ax1.plot(xs, ys)




class Write():
    def __init__(self, name_file):
        rospy.init_node("graph")
        self.rate = rospy.Rate(10.0)
        self.x = 0
        self.ang = 0
        self.name_file = name_file

    def callback(self, data):
        self.x = data.linear.x 
        self.ang = data.angular.z
        with open('/home/tran/github/bag_file/{}'.format(self.name_file), 'a') as the_file:
            the_file.write('{}, {}\n'.format(self.x, self.ang))
    def spin(self):
        while not rospy.is_shutdown():
            print("Launch file Control first, take data then Ctrl + C  to take graph", self.x)      
            rospy.Subscriber("/cmd_vel_mux/input/teleop", Twist, self.callback)
            self.rate.sleep()

if __name__ =="__main__":
    file1 = Write('file_text.txt')
    file1.spin()
    ani = animation.FuncAnimation(fig, animate, interval=1000) 
    plt.show()