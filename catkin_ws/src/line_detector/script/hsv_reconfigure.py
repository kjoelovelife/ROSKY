#!/usr/bin/env python
import os, sys, yaml
import rospy, rospkg
from std_srvs.srv import EmptyRequest, EmptyResponse, Empty
from dynamic_reconfigure.server import Server
from opencv_apps.cfg import rosky_line_filiterConfig

class HSV_RECONFIGURE(object):
    def __init__(self):
        self.node_name = rospy.get_name()
        self.veh_name = self.node_name.split("/")[1]
        self.yaml_dict = self.readParamFromFile() 
        self.initialize = True   
    
        # create service
        self.srv_save = rospy.Service("~save_calibration", Empty, self.cbSrvSaveCalibration) 

        # start rqt_reconfig
        self.reconfigure = Server(rosky_line_filiterConfig, self.rqt_callback)

    def readParamFromFile(self):
        # Check file existence
        fname = self.getFilePath(self.veh_name)
        default="default"
        # Use default.yaml if file doesn't exsit
        if not os.path.isfile(fname):
            rospy.logwarn("[%s] %s does not exist. Using %s.yaml." %(self.node_name,fname,default))
            fname = self.getFilePath(default)

        with open(fname, 'r') as in_file:
            try:
                yaml_dict = yaml.load(in_file)
                #print(yaml_dict["detector"][1]["configuration"]["hsv_white1"])
                return yaml_dict
            except yaml.YAMLError as exc:
                rospy.logfatal("[%s] YAML syntax error. File: %s fname. Exc: %s" %(self.node_name, fname, exc))
                rospy.signal_shutdown()
                return

    def getFilePath(self, name):
        rospack = rospkg.RosPack()
        return rospack.get_path('rosky_base')+'/config/baseline/line_detector/line_detector_node/' + name + ".yaml"  

    def cbSrvSaveCalibration(self, req):
        file_name = self.getFilePath("rosky01") # self.veh_name
        with open(file_name, 'w') as outfile:
            outfile.write(yaml.dump(self.yaml_dict, default_flow_style=False))
        # Printout
        rospy.loginfo("[%s] Saved to %s" %(self.node_name, file_name))
        return EmptyResponse()

    def rqt_callback(self, config, level):
        if self.initialize == True:
            config.white_h_limit_min = self.yaml_dict["detector"][1]["configuration"]["hsv_white1"][0]
            config.white_s_limit_min = self.yaml_dict["detector"][1]["configuration"]["hsv_white1"][1]
            config.white_v_limit_min = self.yaml_dict["detector"][1]["configuration"]["hsv_white1"][2]
            config.white_h_limit_max = self.yaml_dict["detector"][1]["configuration"]["hsv_white2"][0]
            config.white_s_limit_max = self.yaml_dict["detector"][1]["configuration"]["hsv_white2"][1]
            config.white_v_limit_max = self.yaml_dict["detector"][1]["configuration"]["hsv_white2"][2]

            config.yellow_h_limit_min = self.yaml_dict["detector"][1]["configuration"]["hsv_yellow1"][0]
            config.yellow_s_limit_min = self.yaml_dict["detector"][1]["configuration"]["hsv_yellow1"][1]
            config.yellow_v_limit_min = self.yaml_dict["detector"][1]["configuration"]["hsv_yellow1"][2]
            config.yellow_h_limit_max = self.yaml_dict["detector"][1]["configuration"]["hsv_yellow2"][0]
            config.yellow_s_limit_max = self.yaml_dict["detector"][1]["configuration"]["hsv_yellow2"][1]
            config.yellow_v_limit_max = self.yaml_dict["detector"][1]["configuration"]["hsv_yellow2"][2]

            config.red_1_h_limit_min = self.yaml_dict["detector"][1]["configuration"]["hsv_red1"][0]
            config.red_1_s_limit_min = self.yaml_dict["detector"][1]["configuration"]["hsv_red1"][1]
            config.red_1_v_limit_min = self.yaml_dict["detector"][1]["configuration"]["hsv_red1"][2]
            config.red_1_h_limit_max = self.yaml_dict["detector"][1]["configuration"]["hsv_red2"][0]
            config.red_1_s_limit_max = self.yaml_dict["detector"][1]["configuration"]["hsv_red2"][1]
            config.red_1_v_limit_max = self.yaml_dict["detector"][1]["configuration"]["hsv_red2"][2]

            config.red_2_h_limit_min = self.yaml_dict["detector"][1]["configuration"]["hsv_red3"][0]
            config.red_2_s_limit_min = self.yaml_dict["detector"][1]["configuration"]["hsv_red3"][1]
            config.red_2_v_limit_min = self.yaml_dict["detector"][1]["configuration"]["hsv_red3"][2]
            config.red_2_h_limit_max = self.yaml_dict["detector"][1]["configuration"]["hsv_red4"][0]
            config.red_2_s_limit_max = self.yaml_dict["detector"][1]["configuration"]["hsv_red4"][1]
            config.red_2_v_limit_max = self.yaml_dict["detector"][1]["configuration"]["hsv_red4"][2]

            config.top_cutoff = self.yaml_dict["top_cutoff"]
            config.dilation_kernel_size = self.yaml_dict["detector"][1]["configuration"]["dilation_kernel_size"]
            config.hough_max_line_gap = self.yaml_dict["detector"][1]["configuration"]["hough_max_line_gap"]
            config.hough_min_line_length = self.yaml_dict["detector"][1]["configuration"]["hough_min_line_length"]
            config.canny_thresholds_min = self.yaml_dict["detector"][1]["configuration"]["canny_thresholds"][0]
            config.canny_thresholds_max = self.yaml_dict["detector"][1]["configuration"]["canny_thresholds"][1]
            config.hough_thresholds = self.yaml_dict["detector"][1]["configuration"]["hough_threshold"]

            self.initialize = False
        else:
            hsv_white1 = [config.white_h_limit_min, config.white_s_limit_min, config.white_v_limit_min]
            hsv_white2 = [config.white_h_limit_max, config.white_s_limit_max, config.white_v_limit_max]
            hsv_yellow1 = [config.yellow_h_limit_min, config.yellow_s_limit_min, config.yellow_v_limit_min]
            hsv_yellow2 = [config.yellow_h_limit_max, config.yellow_s_limit_max, config.yellow_v_limit_max]
            hsv_red1 = [config.red_1_h_limit_min, config.red_1_s_limit_min, config.red_1_v_limit_min]
            hsv_red2 = [config.red_1_h_limit_max, config.red_1_s_limit_max, config.red_1_v_limit_max]
            hsv_red3 = [config.red_2_h_limit_min, config.red_2_s_limit_min, config.red_2_v_limit_min]
            hsv_red4 = [config.red_2_h_limit_max, config.red_2_s_limit_max, config.red_2_v_limit_max]
            canny_thresholds = [config.canny_thresholds_min, config.canny_thresholds_max]
            
            self.yaml_dict["detector"][1]["configuration"]["hsv_white1"] = hsv_white1
            self.yaml_dict["detector"][1]["configuration"]["hsv_white2"] = hsv_white2
            self.yaml_dict["detector"][1]["configuration"]["hsv_yellow1"] = hsv_yellow1
            self.yaml_dict["detector"][1]["configuration"]["hsv_yellow2"] = hsv_yellow2
            self.yaml_dict["detector"][1]["configuration"]["hsv_red1"] = hsv_red1
            self.yaml_dict["detector"][1]["configuration"]["hsv_red2"] = hsv_red2
            self.yaml_dict["detector"][1]["configuration"]["hsv_red3"] = hsv_red3
            self.yaml_dict["detector"][1]["configuration"]["hsv_red4"] = hsv_red4
            self.yaml_dict["top_cutoff"] = config.top_cutoff
            self.yaml_dict["detector"][1]["configuration"]["dilation_kernel_size"] = config.dilation_kernel_size
            self.yaml_dict["detector"][1]["configuration"]["hough_max_line_gap"] = config.hough_max_line_gap
            self.yaml_dict["detector"][1]["configuration"]["hough_min_line_length"] = config.hough_min_line_length
            self.yaml_dict["detector"][1]["configuration"]["canny_thresholds"] = canny_thresholds
            self.yaml_dict["detector"][1]["configuration"]["hough_threshold"] = config.hough_thresholds
            rospy.set_param("/" + self.veh_name + "/line_detector_node/detector", self.yaml_dict["detector"])
            rospy.set_param("/" + self.veh_name + "/line_detector_node/top_cutoff", self.yaml_dict["top_cutoff"])
        return config

    def on_shutdown(self): 
        rospy.is_shutdown=True



if __name__ == "__main__":
    rospy.init_node("hsv_reconfigure", anonymous = False)
    hsv_reconfigure_node = HSV_RECONFIGURE()
    rospy.on_shutdown(hsv_reconfigure_node.on_shutdown)
    rospy.spin()

