#!/usr/bin/env python
from __future__ import print_function
import sys , rospy ,cv2 ,time ,signal ,rospkg ,os.path , yaml , io ,thread
import numpy as np
import camera_info_manager
from std_msgs.msg import String , Header
from sensor_msgs.msg import Image, CompressedImage, CameraInfo
from sensor_msgs.srv import SetCameraInfo, SetCameraInfoResponse
from cv_bridge import CvBridge, CvBridgeError
from rosky_msgs.msg import BoolStamped
# jetson nano camera
from jetcam_ros.csi_camera import CSICamera
from jetcam_ros.utils import bgr8_to_jpeg

class CameraNode:

    def __init__(self):
        '''Initialize ros publisher, ros subscriber'''
        self.node_name = rospy.get_name()
        self.veh_name = self.node_name.split("/")[1]
        # set jetson_camera 
        self.device = rospy.get_param("~capture_device",0)
        self.capture_fps = rospy.get_param("~capture_fps",60)
        self.capture_width = rospy.get_param("~capture_width",1280)
        self.capture_height = rospy.get_param("~capture_height",720)
        self.width = rospy.get_param("~width",224)
        self.height = rospy.get_param("~height",224)
        self.capture_flip = rospy.get_param("~capture_flip",0)  
        
        # grap camera
        try:
            self.camera = CSICamera(device=self.device,width=self.width, height=self.height, capture_width=self.capture_width, capture_height=self.capture_height, capture_fps=self.capture_fps,capture_flip=self.capture_flip)
        except StopIteration:
            pass
        
        
        # For intrinsic calibration
        rospack = rospkg.RosPack()
        self.config = self.setupParam("~config","baseline")
        self.cali_file_folder = rospack.get_path('rosky_base') + "/config/" + self.config + "/calibration/camera_intrinsic/"    
        self.frame_id = rospy.get_namespace().strip('/') + "/camera_optical_frame"

        self.has_published = False
        self.pub_img_raw= rospy.Publisher("~image_raw",Image,queue_size=1)
        self.pub_img_compressed= rospy.Publisher("~image_raw/compressed", CompressedImage,queue_size=1)
        self.sub_switch_high = rospy.Subscriber("~framerate_high_switch", BoolStamped, self.cbSwitchHigh, queue_size=1)

        # Create service (for camera_calibration)
        #self.srv_set_camera_info = rospy.Service("~set_camera_info",SetCameraInfo,self.cbSrvSetCameraInfo)
        self.is_shutdown = False
        self.update_framerate = False
        # Setup timer
        rospy.loginfo("[%s] Initialized." %(self.node_name))
        self.bridge = CvBridge()
        
    def cbSwitchHigh(self, switch_msg):
        print(switch_msg)
        if switch_msg.data and self.framerate != self.framerate_high:
            self.framerate = self.framerate_high
            self.update_framerate = True
        elif not switch_msg.data and self.framerate != self.framerate_low:
            self.framerate = self.framerate_low
            self.update_framerate = True
            
    def startCapturing(self):
        rospy.loginfo("[%s] Start capturing." %(self.node_name))
        while not self.is_shutdown and not rospy.is_shutdown():
            image = self.camera.read()
            jpeg_image = np.array(bgr8_to_jpeg(image)).tostring()
            # Construct image_msg
            # Grab image from stream
            stamp = rospy.Time.now()
            # Generate compressed image
            image_msg = CompressedImage()
            image_msg.format = "jpeg"
            image_msg.data = jpeg_image
            image_msg.header.stamp = stamp
            image_msg.header.frame_id = self.frame_id
            self.pub_img_compressed.publish(image_msg)
            
            if not self.has_published:
                rospy.loginfo("[%s] Published the first image." %(self.node_name))
                self.has_published = True

            rospy.sleep(rospy.Duration.from_sec(0.001))
        rospy.loginfo("[%s] Capture Ended." %(self.node_name))
            
    def setupParam(self,param_name,default_value):
        value = rospy.get_param(param_name,default_value)
        rospy.set_param(param_name,value) #Write to parameter server for transparancy
        rospy.loginfo("[%s] %s = %s " %(self.node_name,param_name,value))
        return value
    
if __name__ == "__main__":
    rospy.init_node("jetson_camera",anonymous=False)
    camera_node = CameraNode()
    thread.start_new_thread(camera_node.startCapturing, ())
    rospy.spin()
