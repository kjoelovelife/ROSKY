#!/usr/bin/env python
import rospy
import numpy as np
import math
import rospkg
import os.path
import yaml
from rosky_msgs.msg import  Twist2DStamped, BoolStamped
from sensor_msgs.msg import Joy
from geometry_msgs.msg import Twist
import time
from __builtin__ import True



class CmdMapper(object):
    def __init__(self):
        self.node_name = rospy.get_name()
        self.veh_name = self.node_name.split("/")[1]
        self.pub_topic_name = self.node_name + "/" + self.veh_name + "/car_cmd"
        rospy.loginfo("[%s] Initializing " %(self.node_name))
        
        # Publications
        self.pub_car_cmd = rospy.Publisher("~car_cmd", Twist2DStamped, queue_size=1)

        # Subscriptions
        self.sub_cmd_ = rospy.Subscriber("/cmd_vel", Twist, self.cbCmd, queue_size=1)
        
        # set steerGain
        self.readParamFromFile()

        # timer
        self.gains_timer = rospy.Timer(rospy.Duration.from_sec(1.0), self.getGains_event)

    def readParamFromFile(self):
        # Check file existence
        fname = self.getFilePath(self.veh_name)
        # Use default.yaml if file doesn't exsit
        if not os.path.isfile(fname):
            rospy.logwarn("[%s] %s does not exist. Using default.yaml." %(self.node_name,fname))
            fname = self.getFilePath("default")

        with open(fname, 'r') as in_file:
            try:
                yaml_dict = yaml.load(in_file)
            except yaml.YAMLError as exc:
                rospy.logfatal("[%s] YAML syntax error. File: %s fname. Exc: %s" %(self.node_name, fname, exc))
                rospy.signal_shutdown()
                return

        # Set parameters using value in yaml file
        if yaml_dict is None:
            # Empty yaml file
            return
        for param_name in ["keyboard_gain" , "keyboard_steerGain"]:
            if param_name == "keyboard_steerGain" :
                self.omega_gain = yaml_dict.get("keyboard_steerGain")
            if param_name == "keyboard_gain" :
                self.speed_gain = yaml_dict.get("keyboard_gain")

    def getFilePath(self, name):
        rospack = rospkg.RosPack()
        return rospack.get_path('rosky_base')+'/config/baseline/calibration/kinematics/' + name + ".yaml"  


    def cbCmd(self, cmd_msg):
        self.cmd = cmd_msg
        self.publishControl()

    def publishControl(self):
        car_cmd_msg = Twist2DStamped()
        cmd_gain = 10
        car_cmd_msg.v = self.cmd.linear.x * cmd_gain * self.speed_gain
        car_cmd_msg.omega = self.cmd.angular.z * self.omega_gain
        self.pub_car_cmd.publish(car_cmd_msg)                                     

    def getGains_event(self, event):
        param_name = "/" + self.veh_name + "/inverse_kinematics_node/"
        speed_gain = rospy.get_param( param_name + "keyboard_gain" ,self.speed_gain )
        omega_gain = rospy.get_param( param_name + "keyboard_steerGain" ,self.speed_gain )

        params_old = (self.speed_gain , self.omega_gain)
        params_new = (speed_gain , omega_gain)

        if params_old != params_new:
            rospy.loginfo("[%s] Gains changed." %(self.node_name))
            rospy.loginfo(" old speed_gain : {} , omega_gain : {} ".format(params_old))
            rospy.loginfo(" new speed_gain : {} , omega_gain : {} ".format(params_new))
            self.speed_gain = speed_gain
            self.omega_gain = omega_gain

if __name__ == "__main__":
    rospy.init_node("cmd_mapper",anonymous=False)
    cmd_mapper = CmdMapper()
    rospy.spin()












