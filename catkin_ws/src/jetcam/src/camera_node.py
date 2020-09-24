#!/usr/bin/env python
#=============================================================================
#MIT License

#Copyright (c) 2020 Lin Wei-Chih

#Permission is hereby granted, free of charge, to any person obtaining a copy
#of this software and associated documentation files (the "Software"), to deal
#in the Software without restriction, including without limitation the rights
#to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
#copies of the Software, and to permit persons to whom the Software is
#furnished to do so, subject to the following conditions:

#The above copyright notice and this permission notice shall be included in all
#copies or substantial portions of the Software.

#THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
#IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
#FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
#AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
#LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
#OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
#SOFTWARE.
#==============================================================================
import os , json, glob, datetime, cv2, time, thread, io
import numpy as np
import rospy
from cv_bridge import CvBridge , CvBridgeError
from sensor_msgs.msg import Image , CompressedImage , CameraInfo
from sensor_msgs.srv import SetCameraInfo , SetCameraInfoResponse
from uuid import uuid1
from jetcam.utils import bgr8_to_jpeg
from jetcam.csi_camera import CSICamera

class CameraNode(object):
    def __init__(self):
        self.node_name = rospy.get_name()
        rospy.loginfo("{} Initializing.....".format(self.node_name))

        self.capture_device = self.setupParam("~capture_device",0)
        self.capture_width  = self.setupParam("~capture_width",640)
        self.capture_height = self.setupParam("~capture_height",480)
        self.capture_fps    = self.setupParam("~capture_fps",30)
        self.capture_flip   = self.setupParam("~capture_flip",0)

        # setup jetson nano camera 
        self.camera = CSICamera(capture_device=self.capture_device , 
                                capture_width=self.capture_width ,
                                capture_height=self.capture_height ,
                                capture_fps=self.capture_fps ,
                                capture_flip=self.capture_flip ,
                               )

        self.is_shutdown = False
        self.update_framerate = False
        self.stream = io.BytesIO()

        self.pub_img= rospy.Publisher("~image/compressed",CompressedImage,queue_size=1)


    def update_image(self,change):
        while not self.is_shutdown and not rospy.is_shutdown():
            image = change['new']
            image = 

    def startCapturing(self):
        rospy.loginfo("[%s] Start capturing." %(self.node_name))
        while not self.is_shutdown and not rospy.is_shutdown():
            self.camera.observe(self.update_image,names='value')

    def setupParam(self,param_name,default_value):
        value = rospy.get_param(param_name,default_value)
        rospy.set_param(param_name,value) # Write to parameter server for transparancy
        rospy.loginfo("{} {} = {} ".format(self.node_name,param_name,value))
        return value

    def onShutdown(self):
        rospy.loginfo("{} Closing camera.".format(self.node_name))
        self.camera.unobserve(self.update_image,names='value')
        self.is_shutdown=True
        rospy.loginfo("{} Shutdown.".format(self.node_name))


if __name__ == '__main__': 
    rospy.init_node('camera_node',anonymous=False)
    camera_node = CameraNode()
    rospy.on_shutdown(camera_node.onShutdown)
    rospy.spin()
























