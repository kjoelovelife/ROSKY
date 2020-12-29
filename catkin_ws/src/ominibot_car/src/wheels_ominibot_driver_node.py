#!/usr/bin/env python
import rospy , time
from rosky_msgs.msg import WheelsCmdStamped, BoolStamped
from ominibot_car.ominibot_car_com import Ominibot_Car #DaguWheelsDriver

class WheelsDriverNode(object):
    def __init__(self):
        self.node_name = rospy.get_name()
        rospy.loginfo("[%s] Initializing " %(self.node_name))
        self.estop=False
        self.board_name = self.setupParam( "~board_name","ominibot_car")
        self.port = "/dev/" + self.board_name
        self.baud = self.setupParam( "~baud", 115200)
        self.motor_encoder = self.setupParam( "~motor_encoder", False)
        self.motor_correct = self.setupParam( "~motor_correct", [0, 0 ,0 ,0])
        self.motor_mode = 0x02 if self.motor_encoder == True else 0x03 
        self.cmd_vel_magnification = 100 if self.motor_encoder == True else 10000
        print(self.cmd_vel_magnification)

        # Setup ominibot car 
        self.driver = Ominibot_Car(self.port,self.baud)
        self.driver.motor_correct(self.motor_correct[0], self.motor_correct[1], self.motor_correct[2], self.motor_correct[3])

        # timer
        self.param_timer = rospy.Timer(rospy.Duration.from_sec(1.0),self.cbParamTimer)

        # Setup publishers
        # add publisher for wheels command wih execution time
        self.msg_wheels_cmd = WheelsCmdStamped()
        self.pub_wheels_cmd = rospy.Publisher("~wheels_cmd_executed",WheelsCmdStamped, queue_size=1)

        # Setup subscribers
        self.control_constant = 1.0
        self.sub_topic = rospy.Subscriber("~wheels_cmd", WheelsCmdStamped, self.cbWheelsCmd, queue_size=1)
        self.sub_e_stop = rospy.Subscriber("~emergency_stop", BoolStamped, self.cbEStop, queue_size=1)
 
    def cbParamTimer(self, event):
        tmp_motor_encoder = rospy.get_param("~motor_encoder", False)
        tmp_motor_correct = rospy.get_param("~motor_correct", [0, 0 ,0 ,0])
        if not tmp_motor_correct == self.motor_correct:
            self.motor_correct = tmp_motor_correct
            self.driver.motor_correct(self.motor_correct[0], self.motor_correct[1], self.motor_correct[2], self.motor_correct[3])
            rospy.loginfo("[%s] ~motor_correct = %s \n" %(self.node_name, self.motor_correct))
        if not tmp_motor_encoder == self.motor_encoder:
            self.motor_encoder = tmp_motor_encoder
            self.motor_mode = 0x02 if self.motor_encoder == True else 0x03 
            self.cmd_vel_magnification = 100 if self.motor_encoder == True else 10000

    def setupParam(self,param_name,default_value):
        value = rospy.get_param(param_name,default_value)
        rospy.set_param(param_name,value) #Write to parameter server for transparancy
        rospy.loginfo("[%s] %s = %s " %(self.node_name,param_name,value))
        return value

    def cbWheelsCmd(self,msg):
        if self.estop:
            self.driver.rosky_diff_drive(left=0,right=0, mode=self.motor_mode)
            return
        self.driver.rosky_diff_drive(left=round(msg.vel_left * self.cmd_vel_magnification) ,right=round(msg.vel_right * self.cmd_vel_magnification) , mode=self.motor_mode)
        # Put the wheel commands in a message and publish
        self.msg_wheels_cmd.header = msg.header
        # Record the time the command was given to the wheels_driver
        self.msg_wheels_cmd.header.stamp = rospy.get_rostime()  
        self.msg_wheels_cmd.vel_left = msg.vel_left
        self.msg_wheels_cmd.vel_right = msg.vel_right
        self.pub_wheels_cmd.publish(self.msg_wheels_cmd)

    def cbEStop(self,msg):
        self.estop=not self.estop
        if self.estop:
            rospy.loginfo("[%s] Emergency Stop Activated")
        else:
            rospy.loginfo("[%s] Emergency Stop Released")

    def on_shutdown(self):
        self.driver.rosky_diff_drive(left=400,right=400, mode=0x03)
        rospy.loginfo("[%s] Shutting down."%(rospy.get_name()))

if __name__ == '__main__':
    # Initialize the node with rospy
    rospy.init_node('wheels_driver_node', anonymous=False)
    # Create the DaguCar object
    node = WheelsDriverNode()
    # Setup proper shutdown behavior 
    rospy.on_shutdown(node.on_shutdown)
    # Keep it spinning to keep the node alive
    rospy.spin()
