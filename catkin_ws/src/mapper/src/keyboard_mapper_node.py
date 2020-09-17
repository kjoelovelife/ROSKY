#!/usr/bin/env python
import rospy
import numpy as np
import math
from rosky_msgs.msg import  Twist2DStamped, BoolStamped
from sensor_msgs.msg import Joy
from geometry_msgs.msg import Twist
import time
from __builtin__ import True



class keyboardMapper(object):
    def __init__(self):
        self.node_name = rospy.get_name()
        self.param_veh = rospy.get_param(self.node_name + "/veh", 'rosky')
        rospy.loginfo("[%s] Initializing " %(self.node_name))
        
        # Publications
        self.pub_car_cmd = rospy.Publisher("~car_cmd", Twist2DStamped, queue_size=1)

        # Subscriptions
        self.sub_cmd_ = rospy.Subscriber("~cmd_vel", Twist, self.cbCmd, queue_size=1)
        
        # timer
        self.param_timer = rospy.Timer(rospy.Duration.from_sec(1.0),self.cbParamTimer)
        self.v_gain = self.setupParam("~speed_gain", 1.0) #0.41
        self.omega_gain = self.setupParam("~steer_gain", 1.0) #8.3
        self.keyboard_gain = self.setupParam("~keyboard_gain", 1.0)
        self.keyboard_steerGain = self.setupParam("~keyboard_steerGain", 1.0)
        self.v_gain_default = 0.5
        self.omega_gain_default = 1.0

    def cbParamTimer(self,event):
        self.v_gain = rospy.get_param("~speed_gain", 1.0)
        self.omega_gain = rospy.get_param("~steer_gain", 1.0)

    def setupParam(self,param_name,default_value):
        value = rospy.get_param(param_name,default_value)
        rospy.set_param(param_name,value) #Write to parameter server for transparancy
        rospy.loginfo("[%s] %s = %s " %(self.node_name,param_name,value))
        return value

    def cbCmd(self, cmd_msg):
        self.cmd = cmd_msg
        self.publishControl()

    def publishControl(self):
        car_cmd_msg = Twist2DStamped()
        #car_cmd_msg.header.stamp = self.joy.header.stamp
        car_cmd_msg.v = self.cmd.linear.x * self.v_gain #Left stick V-axis. Up is positive
        car_cmd_msg.omega = self.cmd.angular.z * self.omega_gain
        # setup parameter for inverse_kinematics_node.py use
        if self.cmd.linear.x != 0 and self.cmd.linear.x != self.v_gain_default : 
            self.keyboard_gain = rospy.set_param("~keyboard_gain", car_cmd_msg.v)
        if self.cmd.angular.z != 0 and self.cmd.angular.z != self.omega_gain_default : 
            self.keyboard_steerGain = rospy.set_param("~keyboard_steerGain", car_cmd_msg.omega)

        self.pub_car_cmd.publish(car_cmd_msg)                                     

if __name__ == "__main__":
    rospy.init_node("keyboard_mapper",anonymous=False)
    keyboard_mapper = keyboardMapper()
    rospy.spin()

