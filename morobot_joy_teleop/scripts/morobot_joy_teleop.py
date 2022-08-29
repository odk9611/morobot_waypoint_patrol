#!/usr/bin/env python
# -*- coding: utf-8 -*-

import numpy as np
import rospy
import roslib
import subprocess
import time
from geometry_msgs.msg  import Twist
from sensor_msgs.msg import Joy
import sys
import signal

def signal_handler(signal, frame): # ctrl + c -> exit program
        print('You pressed Ctrl+C!')
        sys.exit(0)
signal.signal(signal.SIGINT, signal_handler)
''' class '''
class robot():
    def __init__(self):
        rospy.init_node('robot_controller', anonymous=True)
        self.velocity_publisher = rospy.Publisher('/cmd_vel', Twist, queue_size=1)
        self.pose_subscriber2 = rospy.Subscriber('/joy',Joy,self.callback)
        self.rate = rospy.Rate(20)

    def callback(self, data):
        global inn
        inn=0
        self.joy = data.buttons
        self.joy2= data.axes
        if np.shape(self.joy)[0]>0:
            inn=1
            self.nemo=self.joy[3]
            self.semo=self.joy[2]
            self.one=self.joy[1]
            self.x=self.joy[0]
            self.R1=self.joy[5]
        if np.shape(self.joy2)[0]>0:
            inn=1
            self.linear=self.joy2[1]
            self.angular=self.joy2[0]
        if inn==1:
            if self.joy[0]==0 and self.joy[1]==0 and self.joy[2]==0 and self.joy[3]==0 and self.joy[5]==0 and self.joy2[0]==0 and self.joy2[1]==0:
                inn=0
            else:
                pass
    def moving(self,vel_msg):
        self.velocity_publisher.publish(vel_msg)

data=Joy()
vel_msg=Twist()

''' robot position '''
morobot = robot()
morobot.callback(data) #without this, getting error
global inn
inn=0

''' main '''
if __name__ == '__main__':
 while 1:
     if inn==1:
        if morobot.R1==1:
             vel_msg.linear.x=morobot.linear*0.22
             vel_msg.angular.z=morobot.angular*1.0
        morobot.moving(vel_msg)        
     morobot.rate.sleep()
