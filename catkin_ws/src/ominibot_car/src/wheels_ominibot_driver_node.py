#!/usr/bin/env python
import rospy , time
from duckietown_msgs.msg import WheelsCmdStamped, BoolStamped
from dagu_car.smart_robotV12_driver import smart_robotV12 #DaguWheelsDriver

class WheelsDriverNode(object):
    def __init__(self):
        self.node_name = rospy.get_name()
        rospy.loginfo("[%s] Initializing " %(self.node_name))
        self.estop=False
        self.board_name = rospy.get_param("/board_name","smart_robot_omnibotV12")
        self.port = "/dev/" + self.board_name
        self.baud = 115200

        # Setup publishers
        self.driver = smart_robotV12(self.port,self.baud)
        self.driver.connect()
       
        # set system node  ===============================
        # vehicle       (Bit0)  : 0 -> omnibot   ; 1 -> Mecanum ; 2 --> encoder , angle param and no imu(1C) ; 3 --> encoder , no angle parameter and no imu(1D) ( Ominibot V0.7 ) ; 
        #                         4 -->  no encoder , angle param , and no imu(1D) ; 5 --> no encoder , no angle parameter , and no imu(1D)
        # imu           (Bit3)  : 0 -> not to do , 1 -> do it
        # imu_axis      (Bit4)  : 0 -> not to do , 1 -> do it
        # return_encoder(Bit6)  : 0 -> not to do , 1 -> do it
        # command       (Bit7)  : 0 -> control   , 1 -> APP
        # motor_direct  (Bit8)  : 0 -> normal    , 1 -> reverse
        # encoder_direct(Bit9)  : 0 -> normal    , 1 -> reverse
        # turn_direct   (Bit10) : 0 -> normal    , 1 -> reverse
        # imu_reverse   (Bit11) : 0 -> normal    , 1 -> reverse    
        #================================================
   
        for i in range(2):
            self.driver.set_system_mode(vehicle=3,imu=0,
                                        imu_axis=0,return_encoder=0,
                                        command=0,motor_direct=0,
                                        encoder_direct=0,turn_direct=0,
                                        imu_reverse=0) # use Ominibot V0.6
            time.sleep(0.3)

        #add publisher for wheels command wih execution time
        self.msg_wheels_cmd = WheelsCmdStamped()
        self.pub_wheels_cmd = rospy.Publisher("~wheels_cmd_executed",WheelsCmdStamped, queue_size=1)

        # Setup subscribers
        self.control_constant = 1.0
        self.sub_topic = rospy.Subscriber("~wheels_cmd", WheelsCmdStamped, self.cbWheelsCmd, queue_size=1)
        self.sub_e_stop = rospy.Subscriber("~emergency_stop", BoolStamped, self.cbEStop, queue_size=1)

    def setupParam(self,param_name,default_value):
        value = rospy.get_param(param_name,default_value)
        rospy.set_param(param_name,value) #Write to parameter server for transparancy
        rospy.loginfo("[%s] %s = %s " %(self.node_name,param_name,value))
        return value

    def cbWheelsCmd(self,msg):
        if self.estop:
            self.driver.TT_motor_knightcar(left=0.0,right=0.0)
            return
        self.driver.TT_motor_knightcar(left=msg.vel_left,right=msg.vel_right)
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
        self.driver.TT_motor_knightcar(left=0.0,right=0.0)
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
