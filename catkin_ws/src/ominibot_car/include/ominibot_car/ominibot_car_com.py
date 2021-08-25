#!/usr/bin/env python
# coding=UTF-8

# Copyright (c) 2020, iCShop, Inc.
# All rights reserved.
#
# Developer: Lin Wei-Chih, kjoelovelife@gmail.com, on 2020-12-22
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions are met:
#
#    * Redistributions of source code must retain the above copyright
#      notice, this list of conditions and the following disclaimer.
#    * Redistributi ons in binary form must reproduce the above copyright
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

## Import Libraries
import numpy as np
import time, struct, binascii, math, threading, sys
from serial import Serial, SerialException
from functools import reduce

class Ominibot_Car(object):
    def __init__(self, port="ominibot_car", baud=115200, timeout=None, py_version=3):
        ## setup connected parameter
        self.version = self.__version__()
        self.param = {
             "port": port,
             "baud": baud,
             "timeout": timeout,
             "send_interval": 0.1,
             "imu_freq": 100,
             "encoder_freq": 25,
             "battery_freq": 1,
             "interrupt_time": 1.5,
             "motor_correct": (0, 0, 0, 0),
        }
        self._serialOK = False
        self._is_synced = False
        self._imu_new_data = False
        self._odom_new_data = False
        self._battery_new_data = False
        self._first_odom = True
        self._first_battery = True
        self.error_flag = False
        self.t_stop = threading.Event()
        try:
            print("Opening serial port: {}".format(self.param["port"]))
            self.connect()
        except SerialException as error:
            print(error)
            raise
            return
        
        ###### auto return value ######
        self.imu  = {"accel":[0, 0, 0], "gyro":[0, 0, 0]}
        self.imu_bfr  = {"accel":[0, 0, 0], "gyro":[0, 0, 0]} 
        self.odom = [0, 0, 0, 0]
        self.odom_bfr = [0, 0, 0, 0]
        self.battery = [0, 0, 0]
        self.battery_bfr = [0, 0, 0]
        self.imu_seq = 0
        self.odom_seq = 0
        self.battery_seq = 0
        self.last_imu_seq = 0
        self.last_odom_seq = 0
        self.last_battery_seq = 0

        ###### read value ######
        self.system_value = {
            "speed_limit"    : [0, 0, 0, 0],
            "location_limit" : [0, 0, 0, 0],
            "location_kp"    : [0, 0, 0, 0],
            "location_ki"    : [0, 0, 0, 0],
            "location_kd"    : [0, 0, 0, 0],
            "speed_kp"       : [0, 0, 0, 0],
            "speed_ki"       : [0, 0, 0, 0],
            "gyro_compensate": [0, 0, 0, 0],
            "system_mode"    : [0, 0, 0, 0],
            "gyro_correct"   : [0, 0, 0, 0],
            "motor_voltage"  : [0, 500],
            "battery_voltage" : [32767, 32767], 
            "gyro_turn_angle": [0, 0, 0, 0],
        }
        if py_version == 3:
            self.respond = {
                "head": 0x23,
                "auto_head": 0xFF,
                "speed_limit": 0x01,
                "location_limit": 0x02,
                "location_kp": 0x03,
                "location_ki": 0x04,
                "location_kd": 0x05,
                "speed_kp": 0x06,
                "speed_ki": 0x07,
                "gyro_compensate": 0x08,
                "system_mode": 0x09,
                "gyro_correct": 0x0A,
                "motor_voltage": 0x0B,
                "battery_voltage": 0x0C,
                "gyro_turn_angle": 0x20,
                "auto_gyro": 0xFA,
                "auto_encoder": 0xFB,
                "auto_battery": 0xFC,
            }
        elif py_version == 2:
            self.respond = {
                "head": '\x23',
                "auto_head":'\xFF',
                "speed_limit": '\x01',
                "location_limit": '\x02',
                "location_kp": '\x03',
                "location_ki": '\x04',
                "location_kd": '\x05',
                "speed_kp": '\x06',
                "speed_ki": '\x07',
                "gyro_compensate": '\x08',
                "system_mode": '\x09',
                "gyro_correct": '\x0A',
                "motor_voltage": '\x0B',
                "battery_voltage": '\x0C',
                "gyro_turn_angle": '\x20',
                "auto_gyro": '\xFA',
                "auto_encoder": '\xFB',
                "auto_battery": '\xFC',
            }    
        else:
            print("Please check out your python version. Default use python3.")
            self.respond = {
                "head": 0x23,
                "auto_head": 0xFF,
                "speed_limit": 0x01,
                "location_limit": 0x02,
                "location_kp": 0x03,
                "location_ki": 0x04,
                "location_kd": 0x05,
                "speed_kp": 0x06,
                "speed_ki": 0x07,
                "gyro_compensate": 0x08,
                "system_mode": 0x09,
                "gyro_correct": 0x0A,
                "motor_voltage": 0x0B,
                "battery_voltage": 0x0C,
                "gyro_turn_angle": 0x20,
                "auto_gyro": 0xFA,
                "auto_encoder": 0xFB,
                "auto_battery": 0xFC,
            }


    def connect(self):
        if self._serialOK == False:
            self.serial = Serial(self.param["port"], self.param["baud"], timeout=self.param["timeout"])
            self._serialOK = True

    def disconnect(self):
        if self._serialOK == True:
            print("Try to disconnect ominibot car")
            self.serial.close()
            self._serialOK == False
            print("Done with disconnecting ominibot car!")

    def serial_thread(self):
        # Serial initialization
        print("========= Serial thread ==========")
        while(not self.t_stop.is_set()):          
            try:
                reading = self.serial.read(2)
                #print(binascii.hexlify(reading))
            except Exception as error:
                self.error_flag = True
                break

            #====== imu data packet (python3) ======#
            if reading[0] == self.respond["auto_head"] and reading[1] == self.respond["auto_gyro"]:
                #ser_in = self.serial.read(13)
                try:
                    ser_in = self.serial.read(13)
                except Exception:
                    self.error_flag = True
                    break
                self.imu_decode(ser_in, 13)
                self._is_synced = True
                #debug
                #to_hex = lambda x: "".join("{02X}".format(ord(c)) for c in reading)
                #print(to_hex(b'\x03\xac23\n'))

            #====== encoder data packet ======#
            elif reading[0] == self.respond["auto_head"] and reading[1] == self.respond["auto_encoder"]:
                #ser_in = self.serial.read(9)
                try:
                    ser_in = self.serial.read(9)
                except Exception:
                    self.error_flag = True
                    break
                self.odom_decode(ser_in, 7)
                self._is_synced = True
            
            #====== battery data packet ======#
            elif reading[0] == self.respond["auto_head"] and reading[1] == self.respond["auto_battery"]:
                #ser_in = self.serial.read(5)
                try:
                    ser_in = self.serial.read(5)
                except Exception:
                    self.error_flag = True
                    break
                self.battery_decode(ser_in, 5)
                self._is_synced = True

            #====== lost sync ======#
            else:
                if self._is_synced == True:
                    if self._first_odom == True or self._first_battery == True:
                        print("Initial syncing...")
                        self._is_synced = False
                        continue
                #print("out of sync")
                #to_hex = lambda x: "".join("{02X}".format(ord(c)) for c in reading)
                #print(to_hex(b'\x03\xac23\n'))

                bfr = self.serial.read(1)
                #to_hex = lambda x: "".join("{02X}".format(ord(c)) for c in bfr)
                #print(to_hex(b' ', end=''))
                self._is_synced = False
        
        # if loop breaks with an error flag
        if self.error_flag == True:
            print("serial read error")
            self.serial.close()
            self._serialOK = False
            self._is_synced = False
            self._odom_new_data = False
            self._battery_new_data = False
            print("thread ends")
            raise
            return
        
        # if threads ends here
        print("Sending stoping signal to ominibot car")
        self.serial.close()
        self._serialOK = False
        self._is_synced = False
        self._odom_new_data = False
        self._imu_new_data = False
        self._battery_new_data = False
        print("thread ends")

    ###### Decode imu data ######
    def imu_decode(self, data, size):
        # reference: https://docs.python.org/3/library/struct.html
        self.imu_bfr["accel"][0] = struct.unpack('>h', data[0: 2])[0]
        self.imu_bfr["accel"][1] = struct.unpack('>h', data[2: 4])[0]
        self.imu_bfr["accel"][2] = struct.unpack('>h', data[4: 6])[0]
        self.imu_bfr["gyro"][0] = struct.unpack('>h', data[6: 8])[0]
        self.imu_bfr["gyro"][1] = struct.unpack('>h', data[8: 10])[0]
        self.imu_bfr["gyro"][2] = struct.unpack('>h', data[10: 12])[0]
        self.imu_seq = struct.unpack('>B', data[12: 13])[0]
        #debug
        #print("imu",self.imu_seq)
        self.imu = self.imu_bfr
        self._imu_new_data = True

    ###### Decode odometry data ######
    def odom_decode(self, data, size):
        # reference: https://docs.python.org/3/library/struct.html
        self.odom_bfr[0] = struct.unpack('>h', data[0: 2])[0]
        self.odom_bfr[1] = struct.unpack('>h', data[2: 4])[0]
        self.odom_bfr[2] = struct.unpack('>h', data[4: 6])[0]
        self.odom_bfr[3] = struct.unpack('>h', data[6: 8])[0]
        self.odom_seq = struct.unpack('>B', data[8: 9])[0]
        #debug
        #print("odom", self.odom_seq, self.odom[0:4])
        if (self.odom_seq != ((self.last_odom_seq + 1 )%256)):
            if not self._first_odom:
                print("odom seq mismatch, prev: {}, now: {}".format(self.last_odom_seq, self.odom_seq))
        if self._first_odom == True:
            self._first_odom = False
        self.last_odom_seq = self.odom_seq
        self.odom = self.odom_bfr
        self._odom_new_data = True

    ###### Decode battery data ######
    def battery_decode(self, data, size):
        # reference: https://docs.python.org/3/library/struct.html
        self.battery_bfr[0] = struct.unpack('>h', data[0: 2])[0]
        self.battery_bfr[1] = struct.unpack('>h', data[2: 4])[0]
        self.battery_seq = struct.unpack('B', data[4: 5])[0]

        #debug
        #print("battery, voltage:{}, power:{}".format(self.battery_bfr[0], self.battery_bfr[1]))
        if (self.battery_seq != ((self.last_battery_seq + 1 )%256)):
            if not self._first_battery:
                print("battery seq mismatch, prev:{}, now:{}".format(self.last_battery_seq,self.battery_seq))
        if self._first_battery:
            self._first_battery = False
        
        self.last_battery_seq = self.battery_seq
        self.battery = self.battery_bfr
        self._battery_new_data = True

    ###### read data 1byte decode ######
    def read_data_decode_1byte(self, param_name, data, size):
        for number in range(size):
            self.system_value[param_name][number] = struct.unpack('B', data[ number: (number + 1) ])[0]
        return self.system_value[param_name]

    ###### read data 2byte decode ######
    def read_data_decode_2byte(self, param_name, data, size):
        for number in range(int(size/2)):
            self.system_value[param_name][number] = struct.unpack('>H', data[ int((number * 2)): int(math.pow(2, number+1)) ])[0]
        return self.system_value[param_name]

    ###### read system mode decode ######
    def system_mode_decode(self, data, size):
        for number in range(size):
            self.system_value["system_mode"][number] = struct.unpack('B', data[ number: (number + 1) ])[0] 

    ###### read motor voltage decode ######
    def motor_voltage_decode(self, data, size):
        self.system_value["motor_voltage"][0] = struct.unpack('>H', data[0: 2])[0]
        self.system_value["motor_voltage"][1] = struct.unpack('>H', data[2: 4])[0]

    ###### read motor voltage decode ######
    def cutoff_voltage_decode(self, data, size):
        self.system_value["cutoff_voltage"][0] = struct.unpack('>H', data[0: 2])[0]
        self.system_value["cutoff_voltage"][1] = struct.unpack('>H', data[2: 4])[0]

    ######## Module communication from outside ######
    def serialOK(self):
        return self._serialOK

    def imu_new_data(self):
        return self._imu_new_data

    def odom_new_data(self):
        return self._odom_new_data

    def battery_new_data(self):
        return self._battery_new_data
    
    def get_imu_data(self):
        if self._imu_new_data == True:
            # data assign
            self._imu_new_data = False
            return self.imu
        else:
            return None

    def get_odom_data(self):
        if self._odom_new_data == True:
            # data assign
            self._odom_new_data = False 
            return {"seq": self.odom_seq, "pos_dt": self.odom}
        else:
            return None

    def get_battery_data(self):
        if self._battery_new_data == True:
            self._battery_new_data = False
            return {"seq": self.battery_seq, "battery": self.battery}
        else:
            None

    def stop_thread(self):
        self.t_stop.set()
        start = time.time()
        if self._serialOK:
            while self._serialOK:
                if (time.time() - start) > 3:
                    self._serialOK = False
            self.serial.close()

    ############### motor control ##################

    def information(self):
        print("Omnibot car Version: {}.".format(self.__version__()))
        print("Mecanum wheel configure: left_front: motor 1, left_back: motor 4, right_front: motor 2, right_back: motor 4")
        print("Omnibot wheel configure: right_front: motor 2, left_front: motor 3, back: motor 1")
        print("ROSKY wheel configure: according to the Ominibot car information.")

    def motor_correct(self, v1=0, v2=0, v3=0, v4=0, information=False, debug=False):
        self.param["motor_correct"] = (v1, v2 ,v3 ,v4)
        if information == True:
            print("Your motor correct: {}".format(self.param["motor_correct"]))

    
    ## coordinate: ROS transformer
    def omnibot(self, Vx=0.0, Vy=0.0, Vz=0.0, information=False, debug=False, platform="omnibot"):
        # set direction
        function = {
            "Vx": lambda V: 0 if V >= 0 else math.pow(2, 2),
            "Vy": lambda V: 0 if V >= 0 else math.pow(2, 1),
            "Vz": lambda V: 0 if V < 0 else math.pow(2, 0),
        } 
        direction = [
            function["Vx"](Vx),
            function["Vy"](Vy),
            function["Vz"](Vz),
        ]       
        direction = int(reduce(lambda add_x, add_y: add_x + add_y, direction)) 
        Vx = int(round(self.clamp( abs(Vx), 0, 65536 )))
        Vy = int(round(self.clamp( abs(Vy), 0, 65536 )))
        Vz = int(round(self.clamp( abs(Vz), 0, 65536 )))           
        cmd = bytearray(b'\xFF\xFE\x01')
        cmd += struct.pack('>h', Vx) # 2-bytes , velocity for x axis 
        cmd += struct.pack('>h', Vy) # 2-bytes , velocity for y axis 
        cmd += struct.pack('>h', Vz)  # 2-bytes , velocity for z axis       
        # 1-bytes, direction for x(bit2), y(bit1), z(bit0), and 0: normal, 1: reverse
        cmd += struct.pack('>b', direction)
        if debug == True :
            print("send signal about {}: {} ".format(platform, binascii.hexlify(cmd)))
        if self._serialOK == True:       
            self.serial.write(cmd)
            time.sleep(self.param["send_interval"])
    
    def mecanum(self, Vx=0.0, Vy=0.0, Vz=0.0):
        self.omnibot(Vx=Vx, Vy=Vy, Vz=-Vz, platform="mecanum")

    def individual_wheel(self, v1=0.0, v2=0.0, v3=0.0, v4=0.0, mode=0x03,  information=False, debug=False):
        ## mode: 0x02 -> with encoder, 0x03 -> without encoder 
        ## setting up reverse, left motors are normal direction, right motors are reverse direction 
        function = {
            "v1": lambda V: math.pow(2, 2) if V < 0 else 0,
            "v2": lambda V: math.pow(2, 1) if V < 0 else 0,
            "v3": lambda V: math.pow(2, 0) if V < 0 else 0,
            "v4": lambda V: math.pow(2, 3) if V < 0 else 0,
        }
        direction = [
            function["v1"](v1),
            function["v2"](v2),
            function["v3"](v3),
            function["v4"](v4),
        ]
        direction = int(reduce(lambda add_x, add_y: add_x + add_y, direction))
        if mode == 0x02:
            speed_max = 100
            speed_min = 0
        elif mode == 0x03:
            speed_max = 10000
            speed_min = 0
        else:
            print("Mode error! Please chechout your setting(just 0x02 or 0x03).")
        speed = {
            "v1":int(round(self.clamp(abs(v1) + self.param["motor_correct"][0], speed_min, speed_max))),
            "v2":int(round(self.clamp(abs(v2) + self.param["motor_correct"][1], speed_min, speed_max))),
            "v3":int(round(self.clamp(abs(v3) + self.param["motor_correct"][2], speed_min, speed_max))),
            "v4":int(round(self.clamp(abs(v4) + self.param["motor_correct"][3], speed_min, speed_max))),
        }
        ## setting up wheel velocity
        cmd = bytearray(b'\xFF\xFE')
        cmd.append(mode)
        cmd += struct.pack('>h', speed["v1"])  # 2-bytes
        cmd += struct.pack('>h', speed["v2"])   # 2-bytes
        cmd += struct.pack('>h', speed["v3"])  # 2-bytes
        cmd += struct.pack('>h', speed["v4"])   # 2-bytes     
        cmd += struct.pack('>b', direction) # 1-bytes 
        if debug == True :
            print("send signal about individual_wheel: {} ".format(binascii.hexlify(cmd)))
        if self._serialOK == True:       
            self.serial.write(cmd)
            time.sleep(self.param["send_interval"])

    def rosky_diff_drive(self, left=0.0, right=0.0, alpha=-1, mode=0x02, magnification=1, information=False, debug=False):
        # mode : 0x02 -> with encoderm 0x03 -> without encoder
        # V1: rf, V2: lf, V3: rb, V4: lb
        speed_limit = {
            "max": 100 if mode == 0x02 else 10000,
            "min":0,
        }
        left = left if mode == 0x03 else left * alpha
        right = right if mode == 0x03 else right * alpha
        ## setting up reverse, left motors are normal direction, right motors are reverse direction 
        function = {
            "right": lambda V: math.pow(2, 0) + math.pow(2, 2)  if V < 0 else 0,
            "left" : lambda V: 0 if V < 0 else math.pow(2, 1) + math.pow(2, 3),
        }
        direction = [
            function["right"](right),
            function["left"](left)
        ]
        direction = int(reduce(lambda add_x, add_y: add_x + add_y, direction))
        ## setting up wheel velocity
        speed = {
            "v1": int(round(self.clamp(abs( (right * magnification)) + self.param["motor_correct"][0], speed_limit["min"], speed_limit["max"]))),
            "v2": int(round(self.clamp(abs( (left  * magnification)) + self.param["motor_correct"][1], speed_limit["min"], speed_limit["max"]))),
            "v3": int(round(self.clamp(abs( (right * magnification)) + self.param["motor_correct"][2], speed_limit["min"], speed_limit["max"]))),
            "v4": int(round(self.clamp(abs( (left  * magnification)) + self.param["motor_correct"][3], speed_limit["min"], speed_limit["max"]))),
        }
        cmd = bytearray(b'\xFF\xFE')
        cmd.append(mode)
        cmd += struct.pack('>h',speed["v1"])   # 2-bytes
        cmd += struct.pack('>h',speed["v2"])   # 2-bytes
        cmd += struct.pack('>h',speed["v3"])   # 2-bytes
        cmd += struct.pack('>h',speed["v4"])   # 2-bytes     
        cmd += struct.pack('>b',direction) # 1-bytes 
        if debug == True :
            print("send signal about rosky_diff_drive: {} ".format(binascii.hexlify(cmd)))
        if self._serialOK == True:       
            self.serial.write(cmd)
            time.sleep(self.param["send_interval"])

    def clamp(self,value=0.0, _min=0.0, _max=0.0):
        return max(min(_max, value), _min) 

    def set_mode_A(self, param_name, number=50, information=False, debug=False):
        item = {
            "load_setup": 0x01,
            "initialize": 0x02,
            "write_setting": 0x03,
            "gyro_compensate_off": 0x04,
            "gyro_compensate_on": 0x05,
            "gyro_compensate_restart": 0x06,
        }
        if param_name in item:
            Tx8 = item.get(param_name)
        else:
            print("Please check out your param name in\n {}".format(list(item.keys())))  
            return  
        # send signal      Tx0 Tx1 Tx2 Tx3 Tx4 Tx5 Tx6 Tx7
        cmd = bytearray(b'\xFF\xFE\x80\x80\x00\x80\x00\x00')
        cmd.append(Tx8)
        cmd.append(0x00)
        if debug == True :
            print("send signal about {}: {} ".format(param_name, binascii.hexlify(cmd)))    
        if self._serialOK == True:   
            for index in range(number):    
                self.serial.write(cmd)
                time.sleep(self.param["send_interval"])
        return

    def load_setup(self, number=50, information=False, debug=False):
        self.set_mode_A(param_name="load_setup", number=number, information=information, debug=debug)

    def initialize(self, number=50, information=False, debug=False):
        self.set_mode_A(param_name="initialize", number=number, information=information, debug=debug)

    def write_setting(self, number=50 ,information=False, debug=False):
        self.set_mode_A(param_name="write_setting", number=number, information=information, debug=debug)

    def gyro_compensate(self, switch="off", number=50, information=False, debug=False):
        param_name = ""
        if switch == "off":
            param_name = "gyro_compensate_off"
        elif switch == "on":
            param_name = "gyro_compensate_on"
        elif switch == "restart":
            param_name = "gyro_compensate_restart"         
        else:
            print("Error. Please restart your code and ominibot car.")
            self.error_flag = True
        self.set_mode_A(param_name=param_name, number=number, information=information, debug=debug)

    def set_mode_B(self, param_name, value_1=0, value_2=0, number=50, information=False, debug=False):
        item = {
            "speed_limit": 0x01,
            "location_limit": 0x02,
            "location_kp": 0x03,
            "location_ki": 0x04,
            "location_kd": 0x05,
            "speed_kp": 0x06,
            "speed_ki": 0x07,
            "gyro_compensate": 0x08,
            "system_mode": 0x09,
            "gyro_correct": 0x0A,
            "motor_voltage": 0x0B,
            "battery_voltage": 0x0C,
        }
        item_value_byte_2 = ["speed_limit", "location_limit", "gyro_correct", "motor_voltage"]
        item_value_byte_4 = ["location_kp", "location_ki", "location_kd", "speed_kp", "speed_ki", "gyro_compensate", "system_mode", "location_kp"]
        item_value_2      = ["battery_voltage"]
        if param_name in item:
            Tx4 = item.get(param_name)
        else:
            print("Please check out your param name in\n {}".format(list(item.keys())))  
            return  
        # send signal      Tx0 Tx1 Tx2 Tx3
        cmd = bytearray(b'\xFF\xFE\x80\x80')
        cmd.append(Tx4)
        if param_name in item_value_byte_2:
            cmd.append(0x00) # Tx5
            cmd.append(0x00) # Tx6
            cmd += struct.pack('>H',int(value_1))  # Tx7, Tx8
        elif param_name in item_value_byte_4:
            cmd += struct.pack('>I',int(value_1))  # Tx5, Tx6, Tx7, Tx8
        elif param_name in item_value_2:
            cmd += struct.pack('>H',int(value_1))  # Tx5, Tx6
            cmd += struct.pack('>H',int(value_2))  # Tx7, Tx8
        else:
            print("Error")
        cmd.append(0x00) # Tx9
        if debug == True :
            print("send signal about {}: {} ".format(param_name, binascii.hexlify(cmd)))    
        if self._serialOK == True:  
            print("Setting {}...".format(param_name)) 
            for index in range(number):    
                self.serial.write(cmd)
                time.sleep(self.param["send_interval"])
            if information == True:
                self.read_data(param_name=param_name, information=True)
        return
    
    def set_speed_limit(self, speed, information=False, debug=False):
        self.set_mode_B(param_name="speed_limit", value_1=int(speed), information=information, debug=debug)

    def set_location_limit(self, location, information=False, debug=False):
        self.set_mode_B(param_name="location_limit", value_1=int(location), information=information, debug=debug)
    
    def set_location_PID(self, controller, gain, information=False, debug=False):
        _controller = ["kp", "ki", "kd"]
        if controller in _controller:
            if controller == "kp":
                param_name = "location_kp"
            elif controller == "ki":
                param_name = "location_ki"
            elif controller == "kd":
                param_name = "location_kd"
            else:
                print("Error! Please check out your controller, just can type: {}.".format(_controller))
                return
        self.set_mode_B(param_name=param_name, value_1=int(gain), information=information, debug=debug)

    def set_speed_PI(self, controller, gain, information=False, debug=False):
        _controller = ["kp", "ki"]
        if controller in _controller:
            if controller == "kp":
                param_name = "speed_kp"
            elif controller == "ki":
                param_name = "speed_ki"
            else:
                print("Error! Please check out your controller, just can type: {}.".format(_controller))
                return
        self.set_mode_B(param_name=param_name, value_1=int(gain), information=information, debug=debug)    

    def set_gyro_compensate_param(self, value, information=False, debug=False):
        self.set_mode_B(param_name="gyro_compensate", value_1=int(value), information=information, debug=debug)      

    #================ set system node ===============
    # vehicle       (Bit0)  : 0 -> omnibot, 1-> Mecanum, 2-> individual with encoder, 3-> individual without encoder  
    # imu           (Bit3)  : 0 -> not to do , 1 -> do it
    # imu_axis      (Bit4)  : 0 -> not to do , 1 -> do it
    # motor_direct  (Bit8)  : 0 -> normal    , 1 -> reverse
    # encoder_direct(Bit9)  : 0 -> normal    , 1 -> reverse
    # turn_direct   (Bit10) : 0 -> normal    , 1 -> reverse
    # imu_reverse   (Bit11) : 0 -> normal    , 1 -> reverse    
    #================================================
    def set_system_mode(self, information=False, debug=False, platform=None, vehicle=0, imu_correct=False, imu_axis_correct=False, motor_reverse=False, encoder_reverse=False, turn_reverse=False, imu_reverse=False): 
        _platform = {
                "omnibot": 0,
                "mecanum": 1,
                "four_wheel": 2,
            }
        if platform in _platform.keys():
            vehicle = _platform.get(platform, 0)
        else:
            if platform == None:
                print("Please choose platform: {} ".format(list(_platform)))
                return
            else:
                print("We don't have platform [{}]. Please choose platform below: ".format(platform))
                print(list(_platform.keys()))
                return
        calculate={
            "vehicle"       : lambda setting : setting,
            "imu"           : lambda setting : 0 if setting == False else math.pow(2, 3),
            "imu_axis"      : lambda setting : 0 if setting == False else math.pow(2, 4),
            "motor_direct"  : lambda setting : 0 if setting == False else math.pow(2, 8),
            "encoder_direct": lambda setting : 0 if setting == False else math.pow(2, 9),
            "turn_direct"   : lambda setting : 0 if setting == False else math.pow(2, 10),
            "imu_direct"    : lambda setting : 0 if setting == False else math.pow(2, 11),
        }  
        mode = [
            calculate["vehicle"](vehicle),
            calculate["imu"](imu_correct),
            calculate["imu_axis"](imu_axis_correct),
            calculate["motor_direct"](motor_reverse),
            calculate["encoder_direct"](encoder_reverse),
            calculate["turn_direct"](turn_reverse),
            calculate["imu_direct"](imu_reverse),
        ]
        mode = int(reduce(lambda add_x, add_y: add_x + add_y, mode))
        cmd = bytearray(b'\xFF\xFE\x80\x80\x09\x00\x00') # Tx[0]~Tx[6]
        cmd += struct.pack('>h', mode)                   # Tx[7] ,Tx[8]
        cmd.append(0x00)                                 # Tx[9]
        if debug == True :
            print("send signal about set system mode: {} ".format(binascii.hexlify(cmd)))
        if self._serialOK == True: 
            for index in range(5):       
                self.serial.write(cmd)
                time.sleep(0.01)
            if information == True:
                print("Your platform now setting: {} ".format(platform))   
        return 

    def set_gyro_correct(self, value, information=False, debug=False):
        self.set_mode_B(param_name="gyro_correct", value_1=int(value), information=information, debug=debug)
                
    def set_motor_voltage(self, voltage=5, information=False, debug=False):
        voltage = int(voltage * 100)
        self.set_mode_B(param_name="motor_voltage", value_1=voltage, information=information, debug=debug) 


    def set_battery_voltage(self, full=12.6, cut=11.1, information=False, debug=False):
        full = int(full * 100)
        cut = int(cut * 100) 
        self.set_mode_B(param_name="battery_voltage", value_1=full, value_2=cut, information=information, debug=debug)

    def read_data(self, param_name, information=False, debug=False):
        if param_name in self.system_value:
            if not param_name == "head" or param_name == "auto_head":
                Tx4 = {
                    "speed_limit": 0x11,
                    "location_limit": 0x12,
                    "location_kp": 0x13,
                    "location_ki": 0x14,
                    "location_kd": 0x15,
                    "speed_kp": 0x16,
                    "speed_ki": 0x17,
                    "gyro_compensate": 0x18,
                    "system_mode": 0x19,
                    "gyro_correct": 0x1A,
                    "motor_voltage": 0x1B,
                    "battery_voltage": 0x1C,
                    "gyro_turn_angle": 0x20,
                }.get(param_name, 0)
            else:
                print("Please check out your param name in\n {}".format(list(self.system_value.keys())))  
                return 
        else:
            print("Please check out your param name in\n {}".format(list(self.system_value.keys())))  
            return  
        # send signal      Tx0 Tx1 Tx2 Tx3 
        cmd = bytearray(b'\xFF\xFE\x80\x80') 
        cmd.append(Tx4)    # Tx4
        for index in range(5, 10, 1): # Tx5 ~ Tx9
            cmd.append(0x00)
        if debug == True :
            print("send signal about {}: {} ".format(param_name, binascii.hexlify(cmd)))
        if self._serialOK == True:  
            start = time.time()
            interval = time.time() - start
            _read = False
            while(interval < self.param["interrupt_time"]):
                try:
                    self.serial.write(cmd)
                    reading = self.serial.read(2)
                    if reading[0] == self.respond["head"] and reading[1] == self.respond[param_name]:
                        _read = True
                        break
                    else:
                        interval = time.time() - start
                    time.sleep(0.01)
                except Exception:
                    self.error_flag = True
                    break
            if _read == True:
                try:
                    ser_in = self.serial.read(4)
                except Exception as error:
                    self.error_flag = True
                if  param_name == "motor_voltage" or param_name == "battery_voltage":
                    data_decode = self.read_data_decode_2byte(param_name, ser_in, 4)
                else:
                    data_decode = self.read_data_decode_1byte(param_name, ser_in, 4)
                if information == True:
                    print("{}: {}".format(param_name, data_decode)) 
                else:
                    return data_decode
            else:
                print("Warn! Can not get {}. Please disconnect and try again.".format(param_name))
        return

    def read_speed_limit(self, information=False, debug=False):
        self.read_data(param_name="speed_limit", information=information, debug=debug)

    def read_location_limit(self, information=False, debug=False):
        self.read_data(param_name="location_limit", information=information, debug=debug)

    def read_location_kp(self, information=False, debug=False):
        self.read_data(param_name="location_kp", information=information, debug=debug)

    def read_location_ki(self, information=False, debug=False):
        self.read_data(param_name="location_ki", information=information, debug=debug)

    def read_location_kd(self, information=False, debug=False):
        self.read_data(param_name="location_kd", information=information, debug=debug)

    def read_speed_kp(self, information=False, debug=False):
        self.read_data(param_name="speed_kp", information=information, debug=debug)

    def read_speed_ki(self, information=False, debug=False):
        self.read_data(param_name="speed_ki", information=information, debug=debug)

    def read_gyro_compensate(self, information=False, debug=False):
        self.read_data(param_name="gyro_compensate", information=information, debug=debug)

    def read_system_mode(self, information=False, debug=False):
        self.read_data(param_name="system_mode", information=information, debug=debug)

    def read_gyro_correct(self, information=False, debug=False):
        self.read_data(param_name="gyro_correct", information=information, debug=debug)

    def read_motor_voltage(self, information=False, debug=False):
        self.read_data(param_name="motor_voltage", information=information, debug=debug)

    def read_battery_voltage(self, information=False, debug=False):
        self.read_data(param_name="battery_voltage", information=information, debug=debug) 
   
    def __version__(self, information=False):
        if information == True:
            print("Firmware date       : 2020.12.10.")
            print("Firmware version    : V0.08.")
            print("Firmware system item: {}.".format(list(self.system_value.keys())))
            print("Driver developer    : Wei-Chih Lin, github: https://github.com/kjoelovelife")
            print("More information    : https://github.com/CIRCUSPi/OminiBotHV")
            return
        else:
            return "V0.08"

if __name__ == '__main__':
    #### test code ####
    _port = "/dev/ominibot_car"
    _baud = 115200
    ominibot  = Ominibot_Car(_port,_baud, py_version=3)
    #ominibot.set_system_mode(platform="omnibot")
    #ominibot.__version__(information=True)
    #ominibot.set_battery_voltage(cut=11.1, information=True)
    #ominibot.read_battery_voltage(information=True)
    #ominibot.set_battery_voltage(10.1)
    #motor_driver.set_motor_voltage(7.4)
    #ominibot.gyro_compensate(switch="off", information=True, debug=True)

    ###### auto read information example and control motor ######
    '''
    try:
        thread = threading.Thread(target=motor_driver.serial_thread)
        thread.start()
    except:
        print("error")
        motor_driver.stop_thread()
        sys.exit(0)
    start = time.time()
    end   = time.time()
    interval = end - start
    while(interval<3):
        battery = motor_driver.get_battery_data()
        imu     = motor_driver.get_imu_data()
        odom    = motor_driver.get_odom_data()
        print(battery)
        print(imu)
        print(odom)
        motor_driver.rosky_diff_drive(left=40,right=40, mode=0x02)
        time.sleep(1)
        end = time.time()
        interval = end - start
    motor_driver.stop_thread()
    '''

    ###### motor control example ######

    ominibot.set_system_mode(platform="mecanum")
    start = time.time()
    interval = 0
    while(interval< 10):
        interval = time.time() - start
        if interval < 3:
            speed = [0, 0, 100]
        elif interval > 3 and interval < 7:
            speed = [0, 0, -100]
        else:
            speed = [0, 0, 100]
        ominibot.mecanum(speed[0], speed[1], speed[2])
        
    ominibot.mecanum(0, 0, 0)
    ominibot.disconnect()


