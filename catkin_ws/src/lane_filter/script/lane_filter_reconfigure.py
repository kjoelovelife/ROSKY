#!/usr/bin/env python
import os, sys, yaml
import rospy, rospkg
from std_srvs.srv import EmptyRequest, EmptyResponse, Empty
from dynamic_reconfigure.server import Server
from rosky_msgs.cfg import Lane_Filter_ParamConfig

class Lane_Filiter_Param_RECONFIGURE(object):
    def __init__(self):
        self.node_name = rospy.get_name()
        self.veh_name = self.node_name.split("/")[1]
        self.yaml_dict = self.readParamFromFile() 
        self.initialize = True   
    
        # create service
        self.srv_save = rospy.Service("~save_calibration", Empty, self.cbSrvSaveCalibration) 

        # start rqt_reconfig
        self.reconfigure = Server(Lane_Filter_ParamConfig, self.rqt_callback)

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
        return rospack.get_path('rosky_base')+'/config/baseline/lane_filter/lane_filter_node/' + name + ".yaml"  

    def cbSrvSaveCalibration(self, req):
        file_name = self.getFilePath(self.veh_name) # self.veh_name
        with open(file_name, 'w') as outfile:
            outfile.write(yaml.dump(self.yaml_dict, default_flow_style=False))
        # Printout
        rospy.loginfo("[%s] Saved to %s" %(self.node_name, file_name))
        return EmptyResponse()

    def rqt_callback(self, config, level):
        if self.initialize == True:
            for keys in self.yaml_dict:
                config[keys] = self.yaml_dict[keys]
            self.initialize = False
        else:
            for keys in self.yaml_dict:
                self.yaml_dict[keys] = config[keys]
        return config

    def on_shutdown(self): 
        rospy.is_shutdown=True



if __name__ == "__main__":
    rospy.init_node("lane_filiter_reconfigure", anonymous = False)
    lane_filiter_param_reconfigure_node = Lane_Filiter_Param_RECONFIGURE()
    rospy.on_shutdown(lane_filiter_param_reconfigure_node.on_shutdown)
    rospy.spin()

