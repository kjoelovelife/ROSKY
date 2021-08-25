#!/usr/bin/env python
import rospy , time
from rosky_msgs.msg import Twist2DStamped, BoolStamped
from ominibot_car.ominibot_car_com import Ominibot_Car #DaguWheelsDriver

class WheelsDriverNode(object):
    def __init__(self):
        self.node_name = rospy.get_name()
        rospy.loginfo("[%s] Initializing " %(self.node_name))
        self.estop=False
        self.board_name = self.setupParam( "~board_name","ominibot_car")
        self.port = "/dev/" + self.board_name
        self.baud = self.setupParam( "~baud", 115200)
        self.wheel = self.setupParam( "~wheel", "mecanum") 

        # Setup ominibot car 
        self.driver = Ominibot_Car(self.port, self.baud, py_version=2)
        for i in range(2):
            self.driver.set_system_mode(platform=self.wheel)
        time.sleep(0.5)

        # timer
        # self.param_timer = rospy.Timer(rospy.Duration.from_sec(1.0),self.cbParamTimer)

        # Setup publishers
        # add publisher for wheels command wih execution time
        self.msg_wheels_cmd = Twist2DStamped()
        self.pub_wheels_cmd = rospy.Publisher("~wheels_cmd_executed", Twist2DStamped, queue_size=1)

        # Setup subscribers
        self.control_constant = 1.0
        self.sub_topic = rospy.Subscriber("~wheels_cmd", Twist2DStamped, self.cbWheelsCmd, queue_size=1)
        self.sub_e_stop = rospy.Subscriber("~emergency_stop", BoolStamped, self.cbEStop, queue_size=1)

    def setupParam(self,param_name,default_value):
        value = rospy.get_param(param_name,default_value)
        rospy.set_param(param_name,value) #Write to parameter server for transparancy
        rospy.loginfo("[%s] %s = %s " %(self.node_name,param_name,value))
        return value

    def cbWheelsCmd(self, msg):
        if self.estop:
            self.driver.mecanum(0, 0 ,0)
            return
        self.driver.mecanum(msg.v, 0, msg.omega)
        # Put the wheel commands in a message and publish
        self.msg_wheels_cmd.header = msg.header
        # Record the time the command was given to the wheels_driver
        self.msg_wheels_cmd.header.stamp = rospy.get_rostime()  

    def cbEStop(self,msg):
        self.estop=not self.estop
        if self.estop:
            rospy.loginfo("[%s] Emergency Stop Activated")
        else:
            rospy.loginfo("[%s] Emergency Stop Released")

    def on_shutdown(self):
        self.driver.mecanum(0, 0 ,0)
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
