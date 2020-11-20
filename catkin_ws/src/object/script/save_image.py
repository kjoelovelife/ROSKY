#!/usr/bin/env python

import os, sys, argparse, errno, yaml, time
import cv2, numpy 
import rospy, rospkg, threading
from cv_bridge import CvBridge, CvBridgeError
from sensor_msgs.msg import CompressedImage, Image
from uuid import uuid1
from object.srv import save_action, save_actionResponse, select_label, select_labelResponse, picture_interval, picture_intervalResponse



class Save_Image_Node():

    def __init__(self):
        
        # node information
        rospy.sleep(2)
        self.node_name = rospy.get_name()
        self.veh_name = self.node_name.split("/")[1]
        rospy.loginfo("{}  Initializing......".format(self.node_name))

        # setup param
        self.picture_interval = self.setup_parameter("~picture_interval",0.5)
        self.save_action_status = False
        self.yaml_dict = {}

        # read label
        self.read_param_from_file()
        self.label = self.setup_parameter("~label","default")
        rospy.loginfo("Your image label : {} ".format(self.label))
        rospy.loginfo("If your label is wrong, please change the label.")
        self.sub_msg = rospy.Subscriber("~image/raw",Image,self.convert_image_to_cv2,queue_size=1)
        

        # Prepare ros services
        self.srv_save_image = rospy.Service("~save_image_action",save_action, self.cb_srv_save_image)
        self.srv_select_label = rospy.Service("~select_label",select_label, self.cb_srv_select_label)
        self.srv_picture_interval = rospy.Service("~picture_interval",picture_interval, self.cb_srv_picture_interval)

        # CV_bridge
        self.bridge = CvBridge()

        # directory for image save
        #self.path = os.path.abspath(os.path.join(os.path.dirname(__file__),os.pardir)) # ~/ROSKY/catkin_ws/src/object , use system setting
        self.path = self.getFilePath(self.label)

        # timer
        self.save_image_timer = rospy.Timer(rospy.Duration.from_sec(self.picture_interval),self.cb_save_image_timer)
      
        # done information
        rospy.loginfo("You can start collecting your data!")
        rospy.loginfo("The label(folder) and image you have :")
        rospy.loginfo(self.yaml_dict)

    def getFilePath(self , name):
        rospack = rospkg.RosPack()
        return rospack.get_path('object') + "/image/" + name   

    def convert_image_to_cv2(self,img_msg):
        try:
            # Convert your ROS Image ssage to opencv2
            self.cv2_img = self.bridge.imgmsg_to_cv2(img_msg,desired_encoding="bgr8")
        except CvBridgeError as e:
            print(e) 

    def cb_srv_save_image(self, request):
        if request.value == True:
            self.save_action_status = True
        else:
            self.save_action_status = False
        return save_actionResponse()

    def cb_srv_select_label(self, request):
        if request.value in self.yaml_dict :
            rospy.set_param("~label",request.value)
            self.label = request.value
            self.path = self.getFilePath(self.label)
            rospy.loginfo("You select the label : [{}]".format(self.path))
            rospy.loginfo("Now your image will save in [{}]".format(self.path))
        else:
           rospy.loginfo("You don't have the label(folder) [{}] .".format(request.value))
           rospy.loginfo("Please use  {}  to make the label(floder) for saving data ".format(rospkg.RosPack().get_path('object') + "/script/mkdir.py"))
        return select_labelResponse()   

    def cb_srv_picture_interval(self, request):
        self.picture_interval = request.value
        rospy.loginfo("Picture interval set {} seconds .".format(request.value))
        return picture_intervalResponse() 

    def cb_save_image_timer(self,event):
        if self.save_action_status == True:
            self.save_img(self.cv2_img)

    def save_img(self,img):
        img_path = os.path.join(self.path, str(uuid1())+ '.jpg')
        cv2.imwrite(img_path,img)
        rospy.loginfo("save image in {} ".format(self.path))


    def on_shutdown(self): 
        self.write_to_file()
        rospy.loginfo("{} Close.".format(self.node_name))
        rospy.loginfo("{} shutdown.".format(self.node_name))
        rospy.sleep(1)
        rospy.is_shutdown=True

    def read_param_from_file(self):
        fname = rospkg.RosPack().get_path('object') + "/param/image_label.yaml"
        folder = os.listdir(rospkg.RosPack().get_path('object')+"/image")
        with open(fname, 'r') as in_file:
            try:
                self.yaml_dict = yaml.load(in_file)
                if folder != self.yaml_dict.keys() :
                    rospy.loginfo("Please checkout folder [image] and label in [/param/image_label.yaml]. They are different.")
                    rospy.loginfo("save_image.py will shutdown. Please shutdown the launch file after it(jetson_camera.py still runnung).")
                    sys.exit()
            except yaml.YAMLError as exc:
                print(" YAML syntax  error. File: {}".format(fname))
        if self.yaml_dict != None: 
            for label_name in self.yaml_dict:
                image_count = 0
                for dir_path, dir_names, file_names in os.walk(self.getFilePath(label_name)+ "/"):                   
                    for image in file_names:
                        if image.endswith('jpg') or image.endswith('jpeg') :
                            image_count += 1  
                self.yaml_dict[label_name] = image_count
        else:
            rospy.loginfo("Please use  {}  to make the floder for saving data ".format(rospkg.RosPack().get_path('object') + "/script/mkdir.py"))

    def write_to_file(self):
        fname = rospkg.RosPack().get_path('object') + "/param/image_label.yaml"
        self.read_param_from_file()
        with open(fname, 'w') as outfile:
            outfile.write(yaml.dump(self.yaml_dict, default_flow_style=False))

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
    rospy.spin()
    
    
