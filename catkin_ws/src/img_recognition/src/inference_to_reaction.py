#!/usr/bin/env python
import os, sys, argparse, errno, yaml, time, datetime
import rospy, rospkg
import torch, torchvision, cv2
import numpy as np
from rosky_msgs.msg import WheelsCmdStamped, Twist2DStamped
from img_recognition.msg import Inference
from cv_bridge import CvBridge, CvBridgeError
from jetcam_ros.utils import bgr8_to_jpeg


class Inference_To_Reaction(object):
    def __init__(self):
        self.package = "img_recognition"
        self.node_name = rospy.get_name()
        self.veh_name = self.node_name.split("/")[1]
        rospy.loginfo("{}  Initializing inference_model.py......".format(self.node_name))
     
        # local parameter
        self.confidence = {}

        # ros parameter
        self.confidence_threshold = self.setup_parameter("~confidence_threshold", 0.75) 

        # setup the subscriber
        self.sub_msg_inference = rospy.Subscriber("~inference", Inference, self.inference_analyze, queue_size=1)

    def inference_analyze(self, data):
        if data == None:
            pass
        else:
            zip_data = zip(data.labels, data.confidence)
            self.confidence = dict(zip_data)
            recognition = max(self.confidence, key=self.confidence.get)
            if self.confidence[recognition] > self.confidence_threshold: 
                _reaction = self.reaction(recognition)

    def reaction(self, recognition):
        inference_gain = {
                "linear_velocity": [1, 1, 1], # Vx, Vy, Vz
                "angular_velocity": [1, 1, 1], # Ax, Ay, Az
        } 
        if recognition == "free":
            inference_gain = inference_gain
        elif recognition == "blocked":
            for key in inference_gain.keys():
                for index in range(len(inference_gain[key])):
                    inference_gain[key][index] = 0
        else:
            for key in inference_gain.keys():
                for index in range(len(inference_gain[key])):
                    inference_gain[key][index] = 0
        self.setup_parameter("~inference_gain", inference_gain)

    def on_shutdown(self): 
        rospy.loginfo("{} Close.".format(self.node_name))
        rospy.loginfo("{} shutdown.".format(self.node_name))
        rospy.sleep(1)
        rospy.is_shutdown=True

    def setup_parameter(self, param_name, value):
        # value = rospy.get_param(param_name, default_value)
        # Write to parameter server for transparency
        rospy.set_param(param_name, value)
        rospy.loginfo("[%s] %s = %s " % (self.node_name, param_name, value))
        return value


if __name__ == "__main__" :
    rospy.init_node("inference_to_reaction", anonymous=False)
    inference_to_reaction_node = Inference_To_Reaction()
    rospy.on_shutdown(inference_to_reaction_node.on_shutdown)   
    rospy.spin()
