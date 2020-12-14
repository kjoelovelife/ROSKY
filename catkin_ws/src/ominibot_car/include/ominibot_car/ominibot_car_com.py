#!/usr/bin/env python
# coding=UTF-8

# Copyright (c) 2020, iCShop, Inc.
# All rights reserved.
#
# Developer : Lin Wei-Chih , kjoelovelife@gmail.com , on 2020-12-11
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
    def __init__(self,port="ominibot_car", baud=115200, timeout=None):
        ## setup connected parameter
        self.version = self.__version__()
        self.param = {
             "port": port,
             "baud": baud,
             "timeout": timeout,
             "send_interval": 0.1,
             "imu_freq": 100,
             "encoder_freq": 25,
             "battery_freq": 1 ,
             "interrupt_time": 1.5,
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
        
        self.imu  = {"accel":[0, 0, 0] , "gyro":[0, 0, 0]}
        self.imu_bfr  = {"accel":[0, 0, 0] , "gyro":[0, 0, 0]} 
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
        self.system_mode = [0, 0, 0, 0]
        self.motor_voltage = [0, 500]
        self.cutoff_voltage = [32767, 32767]
        self.read_motor_voltage()
        self.read_cutoff_voltage()

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

            #====== imu data packet ======#
            if reading[0] == 0xFF and reading[1] == 0xFA :
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
            elif reading[0] == 0xFF and reading[1] == 0xFB:
                #ser_in = self.serial.read(9)
                try:
                    ser_in = self.serial.read(9)
                except Exception:
                    self.error_flag = True
                    break
                self.odom_decode(ser_in,7)
                self._is_synced = True
            
            #====== battery data packet ======#
            elif reading[0] == 0xFF and reading[1] == 0xFC:
                #ser_in = self.serial.read(5)
                try:
                    ser_in = self.serial.read(5)
                except Exception:
                    self.error_flag = True
                    break
                self.battery_decode(ser_in,5)
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
        self.imu_bfr["accel"][0] = struct.unpack('>h', data[0:2])[0]
        self.imu_bfr["accel"][1] = struct.unpack('>h', data[2:4])[0]
        self.imu_bfr["accel"][2] = struct.unpack('>h', data[4:6])[0]
        self.imu_bfr["gyro"][0] = struct.unpack('>h', data[6:8])[0]
        self.imu_bfr["gyro"][1] = struct.unpack('>h', data[8:10])[0]
        self.imu_bfr["gyro"][2] = struct.unpack('>h', data[10:12])[0]
        self.imu_seq = struct.unpack('>B', data[12:13])[0]
        #debug
        #print("imu",self.imu_seq)
        self.imu = self.imu_bfr
        self._imu_new_data = True

    ###### Decode odometry data ######
    def odom_decode(self, data, size):
        # reference: https://docs.python.org/3/library/struct.html
        self.odom_bfr[0] = struct.unpack('>h', data[0:2])[0]
        self.odom_bfr[1] = struct.unpack('>h', data[2:4])[0]
        self.odom_bfr[2] = struct.unpack('>h', data[4:6])[0]
        self.odom_bfr[3] = struct.unpack('>h', data[6:8])[0]
        self.odom_seq = struct.unpack('>B', data[8:9])[0]
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
        self.battery_bfr[0] = struct.unpack('>h', data[0:2])[0]
        self.battery_bfr[1] = struct.unpack('>h', data[2:4])[0]
        self.battery_seq = struct.unpack('B', data[4:5])[0]

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

    ###### read system mode decode ######
    def system_mode_decode(self, data, size):
        self.system_mode[0] = struct.unpack('B', data[0:1])[0]
        self.system_mode[1] = struct.unpack('B', data[1:2])[0]
        self.system_mode[2] = struct.unpack('B', data[2:3])[0]
        self.system_mode[3] = struct.unpack('B', data[3:4])[0]

    ###### read motor voltage decode ######
    def motor_voltage_decode(self, data, size):
        self.motor_voltage[0] = struct.unpack('>H', data[0:2])[0]
        self.motor_voltage[1] = struct.unpack('>H', data[2:4])[0]

    ###### read motor voltage decode ######
    def cutoff_voltage_decode(self, data, size):
        self.cutoff_voltage[0] = struct.unpack('>H', data[0:2])[0]
        self.cutoff_voltage[1] = struct.unpack('>H', data[2:4])[0]

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
            return {"seq":self.odom_seq, "pos_dt":self.odom}
        else:
            return None

    def get_battery_data(self):
        if self._battery_new_data == True:
            self._battery_new_data = False
            return {"seq":self.battery_seq, "battery":self.battery}
        else:
            None
    
    def get_system_mode(self):
        return self.system_mode

    def get_motor_voltage(self):
        return self.motor_voltage

    def get_cutoff_voltage(self):
        return self.cutoff_voltage

    def stop_thread(self):
        self.t_stop.set()
        start = time.time()
        if self._serialOK:
            while self._serialOK:
                if (time.time() - start) > 3:
                    self._serialOK = False
            self.serial.close()

    def information(self):
        print("Omnibot car Version: {}.".format(self.__version__()))
        print("Mecanum wheel configure: left_front: motor 1, left_back: motor 4, right_front: motor 2, right_back: motor 4")

    ## coordinate: ROS transformer
    def omnibot(self, Vx=0.0, Vy=0.0, Vz=0.0):
        # set direction
        function = {
            "Vx": lambda V: 0 if V >=0 else math.pow(2,2),
            "Vy": lambda V: 0 if V >=0 else math.pow(2,2),
            "Vz": lambda V: 0 if V >=0 else math.pow(2,2),
        } 
        direction = [
            function["Vx"](Vx),
            function["Vy"](Vy),
            function["Vz"](Vz)
        ]       
        direction = reduce(lambda add_x, add_y: add_x + add_y, direction) 
        Vx = round(self.clamp( abs(Vx), 0, 65536 ))
        Vy = round(self.clamp( abs(Vy), 0, 65536 ))
        Vz = round(self.clamp( abs(Vz), 0, 65536 ))            
        cmd = bytearray(b'\xFF\xFE\x01')
        cmd += struct.pack('>h', Vx) # 2-bytes , velocity for x axis 
        cmd += struct.pack('>h', Vy) # 2-bytes , velocity for y axis 
        cmd += struct.pack('>h', Vz)  # 2-bytes , velocity for z axis       
        # 1-bytes , direction for x(bit2) ,y(bit1) ,z(bit0) ,and 0 : normal , 1 : reverse
        cmd += struct.pack('>b',direction) 
        if self._serialOK == True:       
            self.serial.write(cmd)
            time.sleep(self.param["send_interval"])
    
    def mecanum(self, Vx=0.0, Vy=0.0, Vz=0.0):
        self.omnibot(Vx,Vy,Vz)

    def individual_wheel(self, v1=0.0, v2=0.0, v3=0.0, v4=0.0, mode=0x03):
        ## mode : 0x02 -> with encoder, 0x03 -> without encoder 
        ## setting up reverse , left motors are normal direction, right motors are reverse direction 
        function = {
            "v1": lambda V: math.pow(2,2) if V < 0 else 0,
            "v2": lambda V: math.pow(2,1) if V < 0 else 0,
            "v3": lambda V: math.pow(2,0) if V < 0 else 0,
            "v4": lambda V: math.pow(2,3) if V < 0 else 0,
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
            "v1":int(round(self.clamp(abs(v1), speed_min, speed_max))),
            "v2":int(round(self.clamp(abs(v2), speed_min, speed_max))),
            "v3":int(round(self.clamp(abs(v3), speed_min, speed_max))),
            "v4":int(round(self.clamp(abs(v4), speed_min, speed_max))),
        }
        ## setting up wheel velocity
        cmd = bytearray(b'\xFF\xFE')
        cmd.append(mode)
        cmd += struct.pack('>h',speed["v1"])  # 2-bytes
        cmd += struct.pack('>h',speed["v2"])   # 2-bytes
        cmd += struct.pack('>h',speed["v3"])  # 2-bytes
        cmd += struct.pack('>h',speed["v4"])   # 2-bytes     
        cmd += struct.pack('>b',direction) # 1-bytes 
        #print(binascii.hexlify(speed)) # debug
        if self._serialOK == True:       
            self.serial.write(cmd)
            time.sleep(self.param["send_interval"])

    def rosky_diff_drive(self, left=0.0, right=0.0, mode=0x02):
        # mode : 0x02 -> with encoderm 0x03 -> without encoder
        # V1 : rf, V2 : lf, V3 : rb, V4 : lb
        speed_limit = {
            "max": 100 if mode == 0x02 else 10000,
            "min":0,
            "fricition":0,
        }
        magnification = 1
        alpha = -1
        _left = left if mode == 0x03 else left * alpha
        _right = right if mode == 0x03 else right * alpha
        ## setting up reverse , left motors are normal direction, right motors are reverse direction 
        function = {
            "right": lambda V: math.pow(2,0) + math.pow(2,2)  if V < 0 else 0,
            "left" : lambda V: 0 if V < 0 else math.pow(2,1) + math.pow(2,3),
        }
        direction = [
            function["right"](_right),
            function["left"](_left)
        ]
        direction = int(reduce(lambda add_x, add_y: add_x + add_y, direction))
        ## setting up wheel velocity
        left  = int(round(self.clamp(abs( (left  * magnification) + speed_limit["fricition"]), speed_limit["min"], speed_limit["max"])))
        right = int(round(self.clamp(abs( (right * magnification) + speed_limit["fricition"]), speed_limit["min"], speed_limit["max"])))
        cmd = bytearray(b'\xFF\xFE')
        cmd.append(mode)
        cmd += struct.pack('>h',right)  # 2-bytes
        cmd += struct.pack('>h',left)   # 2-bytes
        cmd += struct.pack('>h',right)  # 2-bytes
        cmd += struct.pack('>h',left)   # 2-bytes     
        cmd += struct.pack('>b',direction) # 1-bytes 
        #print(binascii.hexlify(speed)) # debug
        if self._serialOK == True:       
            self.serial.write(cmd)
            time.sleep(self.param["send_interval"])

    #================ set system node ===============
    # vehicle       (Bit0)  : 0 -> omnibot, 1->Mecanum, 2->individual with encoder, 3->individual without encoder  
    # imu           (Bit3)  : 0 -> not to do , 1 -> do it
    # imu_axis      (Bit4)  : 0 -> not to do , 1 -> do it
    # motor_direct  (Bit8)  : 0 -> normal    , 1 -> reverse
    # encoder_direct(Bit9)  : 0 -> normal    , 1 -> reverse
    # turn_direct   (Bit10) : 0 -> normal    , 1 -> reverse
    # imu_reverse   (Bit11) : 0 -> normal    , 1 -> reverse    
    #================================================
    def set_system_mode(self, platform=None,vehicle=0, imu=0, imu_axis=0, motor_direct=0, encoder_direct=0,turn_direct=0, imu_direct=0): 
        _platform = {
                "omnibot":0,
                "mecanum":1,
                "four_wheel":2,
            }
        if platform in _platform.keys():
            vehicle = _platform.get( platform , 0 )
        else:
            if platform == None:
                pass
            else:
                print("We don't have platform [{}]. Please choose platform below: ".format(platform))
                print(list(_platform.keys()))
                return
        calculate={
            "vehicle"       : lambda setting : setting,
            "imu"           : lambda setting : 0 if setting == 0 else math.pow(2,3),
            "imu_axis"      : lambda setting : 0 if setting == 0 else math.pow(2,4),
            "motor_direct"  : lambda setting : 0 if setting == 0 else math.pow(2,8),
            "encoder_direct": lambda setting : 0 if setting == 0 else math.pow(2,9),
            "turn_direct"   : lambda setting : 0 if setting == 0 else math.pow(2,10),
            "imu_direct"    : lambda setting : 0 if setting == 0 else math.pow(2,11),
        }  
        mode = [
            calculate["vehicle"](vehicle),
            calculate["imu"](imu),
            calculate["imu_axis"](imu_axis),
            calculate["motor_direct"](motor_direct),
            calculate["encoder_direct"](encoder_direct),
            calculate["turn_direct"](turn_direct),
            calculate["imu_direct"](imu_direct),
        ]
        mode = int(reduce(lambda add_x, add_y: add_x + add_y, mode))
        cmd = bytearray(b'\xFF\xFE\x80\x80\x09\x00\x00') # Tx[0]~Tx[6]
        cmd += struct.pack('>h',mode)                    # Tx[7] ,Tx[8]
        cmd.append(0x00)                                 # Tx[9]
        print("Omniboard write setting!")
        if self._serialOK == True:        
            self.serial.write(cmd)
            time.sleep(0.01)
            print("Your platform now setting : {} ".format(platform))
            ## debug
            #print("Send to omniboardV12 : {} ".format(binascii.hexlify(cmd)))

    def read_system_mode(self):
        cmd = bytearray(b'\xFF\xFE\x80\x80\x19\x00\x00\x00\x00\x00') 
        if self._serialOK == True:  
            start = time.time()
            interval = time.time() - start
            _read = False
            while(interval < self.param["interrupt_time"]):
                try:
                    self.serial.write(cmd)
                    reading = self.serial.read(2)
                    time.sleep(0.01)
                    if reading[0] == '\x23' and reading[1] == '\x09':
                        _read = True
                        break
                    else:
                        interval = time.time() - start
                except Exception:
                    self.error_flag = True
                    break
            try:
                ser_in = self.serial.read(4)
            except Exception as error:
                self.error_flag = True
            if _read == True:
                self.system_mode_decode(ser_in,4)
                system_mode = self.get_system_mode()  
                print("system mode: {}".format(system_mode[3:4])) 
            else:
                print("Warn! Can not get systemode. please disconnect and try again.")      



    def clamp(self,value=0.0, _min=0.0, _max=0.0):
        return max(min(_max, value), _min) 

    def load_setup(self):
        cmd = bytearray(b'\xFF\xFE\x80\x80\x00\x80\x00\x00\x01\x00')
        if self._serialOK == True:       
            self.serial.write(cmd)

    def write_setting(self):
        cmd = bytearray(b'\xFF\xFE\x80\x80\x00\x80\x00\x00\x03\x00')
        if self._serialOK == True:
            seconds = 5
            print("Writting now..., please wait {} seconds".format(seconds))
            self.serial.write(cmd)
            time.sleep(seconds)
            print("Done.")
    
    def set_motor_voltage(self,voltage=5):
        _voltage = int(voltage * 100) 
        cmd = bytearray(b'\xFF\xFE\x80\x80\x0B\x00\x00')
        cmd += struct.pack('>h',_voltage)  # 2-bytes
        cmd.append(0x00)
        if self._serialOK == True:
            for number in range(200):
                self.serial.write(cmd)
                time.sleep(0.01)
        self.read_motor_voltage(mode="setup")

    def set_cutoff_voltage(self, cut=11.1, full=12.6):
        full_voltage = int(full * 100)
        cutoff_voltage = int(cut * 100) 
        cmd = bytearray(b'\xFF\xFE\x80\x80\x0C')
        cmd += struct.pack('>H',full_voltage)  # 2-bytes
        cmd += struct.pack('>H',cutoff_voltage)  # 2-bytes
        cmd.append(0x00)
        print("Waitting for setting cutoff voltage...")
        if self._serialOK == True:
            for number in range(200):
                self.serial.write(cmd)
                time.sleep(0.01)
        self.read_cutoff_voltage(mode="setup")
    
    def read_motor_voltage(self, mode="initial"):
        cmd = bytearray(b'\xFF\xFE\x80\x80\x1B\x00\x00\x00\x00\x00') 
        if self._serialOK == True:  
            start = time.time()
            interval = time.time() - start
            _read = False
            while(interval < self.param["interrupt_time"]):
                try:
                    self.serial.write(cmd)
                    reading = self.serial.read(2)
                    if reading[0] == 0x23 and reading[1] == 0x0B:
                        _read = True
                        break
                    else:
                        interval = time.time() - start
                    time.sleep(0.01)
                except Exception:
                    self.error_flag = True
                    break
            try:
                ser_in = self.serial.read(4)
            except Exception as error:
                self.error_flag = True
            if _read == True:
                self.motor_voltage_decode(ser_in,4)
                motor_voltage = self.get_motor_voltage()
                print("motor voltage : {}".format(motor_voltage[1])) 
            else:
                if mode == "setup" or mode == "read":
                    print("Warn! Can not get motor_voltage. please disconnect and try again.") 

    def read_cutoff_voltage(self, mode="initial"):
        cmd = bytearray(b'\xFF\xFE\x80\x80\x1C\x00\x00\x00\x00\x00') 
        if self._serialOK == True:  
            start = time.time()
            interval = time.time() - start
            _read = False
            while(interval < self.param["interrupt_time"]):
                try:
                    self.serial.write(cmd)
                    reading = self.serial.read(2)
                    if reading[0] == 0x23 and reading[1] == 0x0C:
                        _read = True
                        break
                    else:
                        interval = time.time() - start
                    time.sleep(0.01)
                except Exception:
                    self.error_flag = True
                    break
            try:
                ser_in = self.serial.read(4)
            except Exception as error:
                self.error_flag = True
            if _read == True:
                self.cutoff_voltage_decode(ser_in,4)
                cutoff_voltage = self.get_cutoff_voltage()  
                print("full voltage  : {} \ncutoff voltage: {}".format(cutoff_voltage[0],cutoff_voltage[1])) 
            else:
                if mode == "setup" or mode == "read":
                    print("Warn! Can not get cutoff voltage. please disconnect and try again.") 

    def __version__(self):
        return "V0.08"

if __name__ == '__main__':
    #### test code ####
    _port = "/dev/ominibot_car"
    _baud = 115200
    motor_driver  = Ominibot_Car(_port,_baud)
    #motor_driver.set_cutoff_voltage(11.1)
    #motor_driver.set_motor_voltage(7.4)
    #motor_driver.write_setting()

    ###### auto read information example ######
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
        motor_driver.individual_wheel(v1=30,v2=30,v3=20,v4=3003,mode=0x03)
        battery = motor_driver.get_battery_data()
        imu     = motor_driver.get_imu_data()
        odom    = motor_driver.get_odom_data()
        serial_ok = motor_driver.serialOK()
        print(battery)
        print(imu)
        print(odom)
        time.sleep(1)
        end = time.time()
        interval = end - start
    motor_driver.stop_thread()
    '''

    ###### motor control example ######
    start = time.time()
    end   = time.time()
    interval = end - start
    time.sleep(2)
    while(interval< 5):
        # left: left side, right: right side
        # mode=0x02: with encode, mode=0x03: without encode
        motor_driver.rosky_diff_drive(left=400,right=-400, mode=0x02) 
        end = time.time()
        interval = end - start
    motor_driver.rosky_diff_drive(left=0.0,right=0.0)
