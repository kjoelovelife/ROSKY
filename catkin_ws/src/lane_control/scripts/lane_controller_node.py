#!/usr/bin/env python
import rospy , rospkg
import numpy as np
import math , yaml , os
from rosky_msgs.msg import  Twist2DStamped, LanePose

class lane_controller(object):
    def __init__(self):
        self.node_name = rospy.get_name()
        self.veh_name = self.node_name.split("/")[1]
        self.reconfigure_name = "lane_controller_reconfigure"
        self.lane_reading = None

        self.pub_counter = 0

        # Setup parameters
        self.setGains()

        # Publicaiton
        self.pub_car_cmd = rospy.Publisher("~car_cmd",Twist2DStamped,queue_size=1)

        # Subscriptions
        self.sub_lane_reading = rospy.Subscriber("~lane_pose", LanePose, self.cbPose, queue_size=1)

        # safe shutdown
        rospy.on_shutdown(self.custom_shutdown)

        # timer
        self.gains_timer = rospy.Timer(rospy.Duration.from_sec(1.0), self.getGains_event)
        rospy.loginfo("[%s] Initialized " %(rospy.get_name()))

    def setupParameter(self,param_name,default_value):
        value = rospy.get_param(param_name,default_value)
        rospy.set_param(param_name,value) #Write to parameter server for transparancy
        rospy.loginfo("[%s] %s = %s " %(self.node_name,param_name,value))
        return value

    def setGains(self):
        v_bar = 0.5 # nominal speed, 0.5m/s
        steer_gain = 1
        k_theta = -2.0
        k_d = - (k_theta ** 2) / ( 4.0 * v_bar)
        theta_thres = math.pi / 6
        d_thres = math.fabs(k_theta / k_d) * theta_thres
        d_offset = 0


        self.v_bar = v_bar # self.setupParameter("~v_bar",v_bar) # Linear velocity
        self.k_d = k_d # self.setupParameter("~k_d",k_theta) # P gain for theta
        self.k_theta = k_theta # self.setupParameter("~k_theta",k_d) # P gain for d
        self.d_thres = d_thres # self.setupParameter("~d_thres",theta_thres) # Cap for error in d
        self.theta_thres = theta_thres # self.setupParameter("~theta_thres",d_thres) # Maximum desire theta
        self.d_offset = d_offset # self.setupParameter("~d_offset",d_offset) # a configurable offset from the lane position
        self.steer_gain = steer_gain # self.setupParameter("~steer_gain",steer_gain) # a multiple for car_control_cmd omega

    def getGains_event(self, event):
        v_bar = rospy.get_param("/" + self.veh_name + "/" + self.reconfigure_name + "/v_bar", self.v_bar)
        k_d = rospy.get_param("/" + self.veh_name + "/" + self.reconfigure_name + "/k_d", self.k_d)
        k_theta = rospy.get_param("/" + self.veh_name + "/" + self.reconfigure_name + "/k_theta", self.k_theta)
        d_thres = rospy.get_param("/" + self.veh_name + "/" + self.reconfigure_name + "/d_thres", self.d_thres)
        theta_thres = rospy.get_param("/" + self.veh_name + "/" + self.reconfigure_name + "/theta_thres", self.theta_thres)
        d_offset = rospy.get_param("/" + self.veh_name + "/" + self.reconfigure_name + "/d_offset", self.d_offset)
        steer_gain = rospy.get_param("/" + self.veh_name + "/" + self.reconfigure_name + "/steer_gain", self.steer_gain)

        params_old = (self.v_bar,self.k_d,self.k_theta,self.d_thres,self.theta_thres, self.d_offset,self.steer_gain)
        params_new = (v_bar,k_d,k_theta,d_thres,theta_thres, d_offset,steer_gain)

        if params_old != params_new:
            rospy.loginfo("[%s] Gains changed." %(self.node_name))
            rospy.loginfo("old gains, v_var %f, k_d %f, k_theta %f, theta_thres %f, d_thres %f, d_offset %f , steer_gain %f" %(params_old))
            rospy.loginfo("new gains, v_var %f, k_d %f, k_theta %f, theta_thres %f, d_thres %f, d_offset %f , steer_gain %f" %(params_new))
            self.v_bar = v_bar
            self.k_d = k_d
            self.k_theta = k_theta
            self.d_thres = d_thres
            self.theta_thres = theta_thres
            self.d_offset = d_offset
            self.steer_gain = steer_gain

    
    def custom_shutdown(self):
        rospy.loginfo("[%s] Shutting down..." %self.node_name)
        
        # Stop listening
        self.sub_lane_reading.unregister()

        # Send stop command
        car_control_msg = Twist2DStamped()
        car_control_msg.v = 0.0
        car_control_msg.omega = 0.0
        self.publishCmd(car_control_msg)
        rospy.sleep(0.5) #To make sure that it gets published.
        rospy.loginfo("[%s] Shutdown" %self.node_name)


    def publishCmd(self,car_cmd_msg):

        #wheels_cmd_msg = WheelsCmdStamped()
        #wheels_cmd_msg.header.stamp = stamp
        #speed_gain = 1.0
        #steer_gain = 0.5
        #vel_left = (speed_gain*speed - steer_gain*steering)
        #vel_right = (speed_gain*speed + steer_gain*steering)
        #wheels_cmd_msg.vel_left = np.clip(vel_left,-1.0,1.0)
        #wheels_cmd_msg.vel_right = np.clip(vel_right,-1.0,1.0)
        self.pub_car_cmd.publish(car_cmd_msg)
        #self.pub_wheels_cmd.publish(wheels_cmd_msg)

    def cbPose(self,lane_pose_msg):
        #inference_linear_gain = rospy.get_param("/" + self.veh_name + "/inference_to_reaction/inference_gain/linear_velocity" , [1,1,1]) # [Vx, Vy, Vz]
        #inference_angular_gain = rospy.get_param("/" + self.veh_name + "/inference_to_reaction/inference_gain/angular_velocity" , [1,1,1]) # [Ax, Ay, Az]
        #inference_linear_gain = float(inference_linear_gain[0])
        #inference_angular_gain = float(inference_angular_gain[2])
        # self.d_offset = 0.23
        self.lane_reading = lane_pose_msg 

        cross_track_err = lane_pose_msg.d - self.d_offset
        heading_err = lane_pose_msg.phi

        car_control_msg = Twist2DStamped()
        car_control_msg.header = lane_pose_msg.header
        car_control_msg.v = self.v_bar #* self.speed_gain # * inference_linear_gain #Left stick V-axis. Up is positive
        if math.fabs(cross_track_err) > self.d_thres:
            cross_track_err = cross_track_err / math.fabs(cross_track_err) * self.d_thres
        # self.k_d * (0.22/self.v_bar) * self.cross_track_err + self.k_theta * (0.22/self.v_bar) * self.heading_err
        car_control_msg.omega =  self.k_d * cross_track_err + (self.k_theta * heading_err *) self.steer_gain #* inference_angular_gain #Right stick H-axis. Right is negative
        
        # controller mapping issue
        # car_control_msg.steering = -car_control_msg.steering
        # print "controls: speed %f, steering %f" % (car_control_msg.speed, car_control_msg.steering)
        # self.pub_.publish(car_control_msg)
        self.publishCmd(car_control_msg)

        # debuging
        # self.pub_counter += 1
        # if self.pub_counter % 50 == 0:
        #     self.pub_counter = 1
        #     print "lane_controller publish"
        #     print car_control_msg

if __name__ == "__main__":
    rospy.init_node("lane_controller",anonymous=False)
    lane_control_node = lane_controller()
    rospy.spin()
