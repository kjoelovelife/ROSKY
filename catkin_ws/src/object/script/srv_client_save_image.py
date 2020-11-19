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
        
        # wait for service start
        self.start = rospy.wait_for_service("/rosky/save_image/save_image_action")
        self.start = rospy.wait_for_service("/rosky/save_image/select_label")

        # set service client
        self.save_image_action = rospy.ServiceProxy("/rosky/save_image/save_image_action",save_action)
        self.select_label      = rospy.ServiceProxy("/rosky/save_image/select_label",select_label)

        # timer
        self.timer = rospy.Timer(rospy.Duration.from_sec(0.1),self.timer)  

    def call_srv_save_image_action(self):    
        try :
            send_signal = self.save_image_action(True)
            print("Capture!")
            rospy.sleep(1)
            send_signal = self.save_image_action(False)
            print("Stop!")
            rospy.sleep(1)
        except rospy.ServiceException as e :
            print("Service call failed".format(e))

    def call_srv_select_label(self,label="default"):    
        try :         
            send_signal = self.select_label(label)
            rospy.loginfo("Select the label : {}".format(label))
        except rospy.ServiceException as e :
            print("Service call failed".format(e))
            
    def getKey(self):
        tty.setraw(sys.stdin.fileno())
        rlist, _, _ = select.select([sys.stdin], [], [], 0.1) 
        if rlist:
            key = sys.stdin.read(1)
        else:
            key = ''
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
        return key

    def timer(self,event):
        key = self.getKey()
        if key == 'p' or key == 'P':
            self.call = self.call_srv_save_image_action()
        else:
            if key == '\x03':
                self.on_shutdown()

    def on_shutdown(self): 
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
        self.is_shutdown=True
        rospy.sleep(rospy.Duration.from_sec(1.0))

if __name__ == "__main__" :
    settings = termios.tcgetattr(sys.stdin)
    rospy.init_node("srv_client_save_image",anonymous=False)
    srv_call_save_image_action = srv_client_save_image_action()
    rospy.on_shutdown(srv_call_save_image_action.on_shutdown)
    rospy.spin()
    
    
