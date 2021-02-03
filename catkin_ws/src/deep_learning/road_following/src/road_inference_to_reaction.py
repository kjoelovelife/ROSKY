#!/usr/bin/env python
import os, sys, argparse, errno, yaml, time, datetime
import rospy, rospkg
import numpy as np
from road_following.msg import Inference
from road_following.cfg import PID_ControlConfig
from road_following.srv import save_action, save_actionResponse
from rosky_msgs.msg import  Twist2DStamped
from dynamic_reconfigure.server import Server

class Inference_To_Reaction(object):
    def __init__(self):
        self.package = "road_following"
        self.node_name = rospy.get_name()
        self.veh_name = self.node_name.split("/")[1]
        rospy.loginfo("[{}]  Initializing road_inference_to_reaction.py......".format(self.node_name))
        self.start = rospy.wait_for_message("/" + self.veh_name +"/road_model_inference/inference", Inference)
    
        # ros parameter
        self.pid_parameter = self.read_param_from_file(package=self.package, folder="param", file_name=(self.veh_name + "_pid.yaml"))
        for keys in self.pid_parameter:
            self.setup_parameter("~" + keys, self.pid_parameter[keys])

        # local parameter
        self.initialize = True

        # setup the rqt_reconfigure 
        self.reconfigure = Server(PID_ControlConfig, self.set_pid_parameter)

        # setup the subscriber
        self.sub_msg_inference = rospy.Subscriber("~inference", Inference, self.inference_analyze, queue_size=1)

        # setup the publisher
        self.pub_car_cmd = rospy.Publisher("~car_cmd", Twist2DStamped, queue_size=1)

        # setup service
        self.srv_save_pid = rospy.Service("~save_pid", save_action, self.srv_save_pid)

    def getFilePath(self, package, folder, file_name):
        rospack = rospkg.RosPack()
        return rospack.get_path(package) + "/" + folder + "/" + file_name  

    def read_param_from_file(self, package, folder, file_name):
        fname = self.getFilePath(package, folder, file_name)
        if not os.path.isfile(fname):
            if file_name == (self.veh_name + "_pid.yaml"):
                rospy.loginfo("[{}] {} does not exist. Using \"default_pid.yaml\" to load parameter".format(self.node_name, fname))
                fname = self.getFilePath(package, folder, file_name="default_pid.yaml")
        with open(fname, 'r') as in_file:
            try:
                yaml_dict = yaml.load(in_file)
            except yaml.YAMLError as exc:
                rospy.loginfo("[{}] YAML syntax  error. File: {}".format(self.node_name, fname))
            return yaml_dict

    def set_pid_parameter(self, config, level):
        if self.initialize == True:
            for keys in self.pid_parameter:
                config[keys] = self.pid_parameter[keys]
            self.initialize = False
        else:
            for keys in self.pid_parameter:
                self.pid_parameter[keys] = config[keys]
            if config["save_parameter"]:
                self.save_pid_parameter(package=self.package, folder="param", file_name=(self.veh_name + "_pid.yaml"))
            config["save_parameter"] = False
        return config

    def inference_analyze(self, data):
        angle = data.angle
        angle_last = data.angle_last
        pid = angle * self.pid_parameter["steering_gain"] + (angle - angle_last) * self.pid_parameter["steering_kd"]
        steering_value = pid + self.pid_parameter["steering_bias"]

        car_cmd_msg = Twist2DStamped()
        #car_cmd_msg.header.stamp = self.joy.header.stamp 
        car_cmd_msg.v = self.pid_parameter["speed_gain"]
        car_cmd_msg.omega = steering_value
        self.pub_car_cmd.publish(car_cmd_msg)  
        self.pub_msg(car_cmd_msg)
    
    def pub_msg(self, car_cmd_msg):
        self.pub_car_cmd.publish(car_cmd_msg)

    def srv_save_pid(self, request):
        self.save_pid_parameter(package=self.road_following, folder="param", file_name=(self.veh_name + "_pid.yaml"))
        return save_actionResponse

    def save_pid_parameter(self, package, folder, file_name):
        fname = rospkg.RosPack().get_path(package) + "/" + folder + "/" + file_name
        with open(fname, 'w') as outfile:
            outfile.write(yaml.dump(self.pid_parameter, default_flow_style=False))
        rospy.loginfo("[{}] Save parameter in {}.".format(self.node_name, fname))

    def setup_parameter(self, param_name, default_value):
        value = rospy.get_param(param_name, default_value)
        # Write to parameter server for transparency
        rospy.set_param(param_name, value)
        rospy.loginfo("[%s] %s = %s " % (self.node_name, param_name, value))
        return value

    def on_shutdown(self): 
        rospy.loginfo("[{}] Close.".format(self.node_name))
        rospy.loginfo("[{}] shutdown.".format(self.node_name))
        rospy.sleep(1)
        rospy.is_shutdown=True


if __name__ == "__main__" :
    rospy.init_node("road_inference_to_reaction", anonymous=False)
    inference_to_reaction_node = Inference_To_Reaction()
    rospy.on_shutdown(inference_to_reaction_node.on_shutdown)   
    rospy.spin()
