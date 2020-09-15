#!/usr/bin/env python
# coding=UTF-8

# Copyright (c) 2011, Willow Garage, Inc.
# All rights reserved.
#
# Developer : Lin Wei-Chih , kjoelovelife@gmail.com 
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions are met:
#
#    * Redistributions of source code must retain the above copyright
#      notice, this list of conditions and the following disclaimer.
#    * Redistributions in binary form must reproduce the above copyright
#      notice, this list of conditions and the following disclaimer in the
#      documentation and/or other materials provided with the distribution.
#    * Neither the name of the Willow Garage, Inc. nor the names of its
#      contributors may be used to endorse or promote products derived from
#       this software without specific prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
# AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
# IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
# ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
# LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
# CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
# SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
# INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
# CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
# ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
# POSSIBILITY OF SUCH DAMAGE.


## import library
import sys, time , select, tty ,termios ,serial
import numpy as np
from ominibot_driver import ominibot_car

msg_vehicle = '''
---------------------------------------------------------
Please set your vehicle. \n
0 --> omnibot.\n
1 --> Mecanum.\n
2 --> encoder , angle param and no imu(1C).\n
3 --> encoder , no angle parameter and no imu(1D).\n 
4 --> no encoder , angle param , and no imu(1D).\n
5 --> no encoder , no angle parameter , and no imu(1D).
'''

msg_imu = '''
---------------------------------------------------------
Please set your imu while start. \n
0 --> imu don't calibrate.\n
1 --> imu calibrate.
'''

msg_imuAxis = '''
---------------------------------------------------------
Please set your imu while axis x and axis y are zero. \n
0 --> imu don't calibrate.\n
1 --> imu calibrate.
'''

msg_returnEncoder = '''
---------------------------------------------------------
Please set your encoder will return value or not. \n
0 --> encoder will not return value.\n
1 --> encoder will return value.
'''

msg_command = '''
---------------------------------------------------------
Please set your command mode. \n
0 --> control mode.\n
1 --> App mode.
'''

msg_motorDirect = '''
---------------------------------------------------------
Please set your motor direction while moving. \n
0 --> normal moving.\n
1 --> reverse moving.
'''

msg_encoderDirect = '''
---------------------------------------------------------
Please set your encoder direction while moving. \n
0 --> normal moving.\n
1 --> reverse moving.
'''

msg_turnDirect = '''
---------------------------------------------------------
Please set your turn direction while moving. \n
0 --> normal moving.\n
1 --> reverse moving.
'''

msg_imuReverse = '''
---------------------------------------------------------
Please set your imu will be reverse or not. \n
0 --> normal.\n
1 --> reverse.
'''
##  start the process  ##
if __name__ == '__main__':


    ## set serial communication
    port  = "/dev/smart_robot_omnibotV12"     #port = "" for linux
    baud  = 115200
    robot = smart_robotV12(port,baud)
    robot.connect()

    ## set parameter
    mode = {'vehicle':0,'imu':0,'imu_axis':0,'return_encoder':0,'command':0,'motor_direct':0,'encoder_direct':0,'turn_direct':0,'imu_reverse':0 }  

    ## set vehicle
    print(msg_vehicle) 
    mode["vehicle"] = int(input("Kind of vehicle : "))

    ## set imu
    print(msg_imu) 
    mode["imu_axis"] = int(input("mode of imu : "))

    ## set imu axis
    print(msg_imuAxis) 
    mode["imu axis"] = int(input("mode of imu axis : "))

    ## set return_encoder
    print(msg_returnEncoder)
    mode["return_encoder"] = int(input("mode of return_encoder : "))

    ## set mode of command
    print(msg_command)
    mode["command"] = int(input("mode of command : "))

    ## set motor_direct
    print(msg_motorDirect)
    mode["motor_direct"] = int(input("mode of motor_direct : "))

    ## set encoder_direct
    print(msg_encoderDirect)
    mode["encoder_direct"] = int(input("mode of encoder_direct : "))

    ## set turn_direct
    print(msg_turnDirect)
    mode["turn_direct"] = int(input("mode of turn_direct : ")) 

    ## set imu_reverse
    print(msg_imuReverse)
    mode["imu_reverse"] = int(input("mode of imu_reverse : "))   

    ## send serial

    for number in range(3):
        robot.set_system_mode(vehicle=mode["vehicle"],imu=mode["imu"],
                              imu_axis=mode["imu_axis"],return_encoder=mode["return_encoder"],
                              command=mode["command"],motor_direct=mode["motor_direct"],
                              encoder_direct=mode["encoder_direct"],turn_direct=mode["turn_direct"],
                              imu_reverse=mode["imu_reverse"])
        time.sleep(1)

    robot.write_setting()
    robot.read_system_mode()
    robot.disconnect()

