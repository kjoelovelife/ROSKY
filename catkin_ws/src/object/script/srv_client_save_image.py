#!/usr/bin/env python

import os, sys, argparse, errno, yaml, time, select, termios, tty
import cv2, numpy 
import rospy, threading
from cv_bridge import CvBridge, CvBridgeError
from sensor_msgs.msg import CompressedImage, Image
from uuid import uuid1
from object.srv import save_action, save_actionResponse, select_label, select_labelResponse



class srv_client_save_image_action(object):

    def __init__(self):
        
        # node information
        self.node_name = rospy.get_name()
        self.veh_name = self.node_name.split("/")[1]

        # wait for service start
        self.start = rospy.wait_for_service(self.veh_name +"/save_image/save_image_action")
        self.start = rospy.wait_for_service(self.veh_name +"/save_image/select_label") 

        # set service client
        self.save_image_action = rospy.ServiceProxy(self.veh_name + "/save_image/save_image_action",save_action)
        self.select_label = rospy.ServiceProxy(self.veh_name + "/save_image/select_label",save_action)
       
        # ros parameter
        self.label = rospy.get_param(self.veh_name + " /save_image/label","default")

        # local parameter 
        self.picture = False

        # Done information
        rospy.loginfo("Service Start!You can click [p] to save picture.")
        rospy.loginfo("Don't forget check out the label now!")
        rospy.loginfo("Now your label is [{}]".format(self.label))
       
        # timer
        self.timer = rospy.Timer(rospy.Duration.from_sec(1),self.cb_timer)

    def call_srv_save_image_action(self,picture):    
        try :
            if picture == True :
                print("Capture!")
            else:
                print("Stop!")
            send_signal = self.save_image_action(picture)
        except rospy.ServiceException as e :
            print("Service call failed".format(e))

    def cb_timer(self,event):
        _label = rospy.get_param(self.veh_name +"/save_image/label","default")
        if _label != self.label:
            self.label = _label
            rospy.loginfo("Now your label is [{}]".format(self.label))

    def call_srv_select_label(self,label="default"):    
        try :         
            send_signal = self.select_label(label)
            rospy.loginfo("Select the label : {}".format(label))
        except rospy.ServiceException as e :
            print("Service call failed".format(e))
            
    def getKey(self):
        tty.setraw(sys.stdin.fileno())
        rlist, _, _ = select.select([sys.stdin], [], [], None) 
        if rlist:
            key = sys.stdin.read(1)
        else:
            key = ''
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
        return key

    def on_shutdown(self): 
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
        print("shutdown...!")
        rospy.is_shutdown=True


    def setup_parameter(self, param_name, default_value):
        value = rospy.get_param(param_name, default_value)
        # Write to parameter server for transparency
        rospy.set_param(param_name, value)
        rospy.loginfo("[%s] %s = %s " % (self.node_name, param_name, value))
        return value

if __name__ == "__main__" :
    settings = termios.tcgetattr(sys.stdin)
    rospy.init_node("srv_client_save_image",anonymous=False)
    srv_call_save_image_action = srv_client_save_image_action()

    while(1):
        key = srv_call_save_image_action.getKey()
        if key == 'p' or key == 'P':
            srv_call_save_image_action.picture = not srv_call_save_image_action.picture
            call = srv_call_save_image_action.call_srv_save_image_action(srv_call_save_image_action.picture)
        else:
            if key == '\x03':
                rospy.on_shutdown(srv_call_save_image_action.on_shutdown)
                break

    termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
    
    
