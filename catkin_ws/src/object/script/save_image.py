#!/usr/bin/env python

import os, sys, argparse, errno, yaml, time
import cv2, numpy 
import rospy, threading
from cv_bridge import CvBridge, CvBridgeError
from sensor_msgs.msg import CompressedImage, Image
from uuid import uuid1



class Save_Image_Node():

    def __init__(self):
        
        # node information
        self.node_name = rospy.get_name()
        self.veh_name = self.node_name.split("/")[1]
        rospy.loginfo("{}  Initializing......".format(self.node_name))

       
        # read label
        self.label = rospy.get_param("~label","default")
        rospy.loginfo("Your image label : {} ".format(self.label))
        rospy.sleep(5)
        self.sub_msg = rospy.Subscriber("/rosky/jetson_camera/image/raw",Image,self.convert_image_to_cv2,queue_size=1)

        # Prepare services
        #self.srv_save_image = rospy.Service("~save_image", , self.cbSrvSetGain)

        # CV_bridge
        self.bridge = CvBridge()

        # directory for image save
        self.path = os.path.abspath(os.path.join(os.path.dirname(__file__),os.pardir)) # ~/ROSKY/catkin_ws/src/object


        

    def convert_image_to_cv2(self,img_msg):
        try:
            # Convert your ROS Image ssage to opencv2
            cv2_img = self.bridge.imgmsg_to_cv2(img_msg,desired_encoding="bgr8")
            self.save_action(cv2_img)
        except CvBridgeError as e:
            print(e) 

    def save_action(self,cv2_img):
        img_path = os.path.join(self.path + "/image/" + self.label, str(uuid1())+ '.jpg')
        cv2.imwrite(img_path,cv2_img)
        rospy.loginfo("save image {} ".format(img_path))

    def on_shutdown(self): 
        rospy.loginfo("{} Close.".format(self.node_name))
        self.is_shutdown=True
        rospy.loginfo("{} shutdown.".format(self.node_name))
        rospy.sleep(rospy.Duration.from_sec(1.0))

    def setup_parameter(self, param_name, default_value):
        value = rospy.get_param(param_name, default_value)
        # Write to parameter server for transparency
        rospy.set_param(param_name, value)
        rospy.loginfo("[%s] %s = %s " % (self.node_name, param_name, value))
        return value

if __name__ == "__main__" :
    rospy.init_node("save_image",anonymous=False)
    save_image_node = Save_Image_Node()
    rospy.on_shutdown(save_image_node.on_shutdown)
    #rospy.spin()
    
    
