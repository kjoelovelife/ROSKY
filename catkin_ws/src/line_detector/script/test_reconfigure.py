#!/usr/bin/env python

import os, sys, yaml
import rospy, rospkg

from dynamic_reconfigure.server import Server
from opencv_apps.cfg import rosky_line_filiterConfig

class HSV_RECONFIGURE(object):
    def __init__(self):
        self.node_name = rospy.get_name()
        self.veh_name = self.node_name.split("/")[1]   
        self.readParamFromFile()
        self.setup_rosky_line_filiterConfig()
       

        # start rqt_reconfig
        self.reconfigure = Server(rosky_line_filiterConfig, self.callback)

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
                #print(yaml_dict["detector"][1]["configuration"]["hsv_white1"])
            except yaml.YAMLError as exc:
                rospy.logfatal("[%s] YAML syntax error. File: %s fname. Exc: %s" %(self.node_name, fname, exc))
                rospy.signal_shutdown()
                return

    def getFilePath(self, name):
        rospack = rospkg.RosPack()
        return rospack.get_path('rosky_base')+'/config/baseline/line_detector/line_detector_node/' + name + ".yaml"    

    def callback(self, config, level):
        hsv_white1 = [config.white_h_limit_min, config.white_s_limit_min, config.white_v_limit_min]
        hsv_white2 = [config.white_h_limit_max, config.white_s_limit_max, config.white_v_limit_max]
        hsv_yellow1 = [config.yellow_h_limit_min, config.yellow_s_limit_min, config.yellow_v_limit_min]
        hsv_yellow2 = [config.yellow_h_limit_max, config.yellow_s_limit_max, config.yellow_v_limit_max]
        hsv_red1 = [config.red_1_h_limit_min, config.red_1_s_limit_min, config.red_1_v_limit_min]
        hsv_red2 = [config.red_1_h_limit_max, config.red_1_s_limit_max, config.red_1_v_limit_max]
        hsv_red3 = [config.red_2_h_limit_min, config.red_2_s_limit_min, config.red_2_v_limit_min]
        hsv_red4 = [config.red_2_h_limit_max, config.red_2_s_limit_max, config.red_2_v_limit_max]
        return config

    def on_shutdown(self): 
        rospy.is_shutdown=True



if __name__ == "__main__":
    rospy.init_node("hsv_reconfigure", anonymous = False)
    hsv_reconfigure_node = HSV_RECONFIGURE()
    rospy.on_shutdown(hsv_reconfigure_node.on_shutdown)
    rospy.spin()

