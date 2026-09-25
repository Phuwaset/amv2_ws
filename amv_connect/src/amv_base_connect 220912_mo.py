#!/usr/bin/python3
from threading import Thread
from serial import Serial
import time
import math
from ctypes import c_short, c_ubyte
from amv_connect.msg import BaseStatus
import numpy as np

class BaseConnect:

    status_data = BaseStatus()
    data_buffer = b'\x00' 

    def __init__(self, port='/dev/amv_controller', baudrate='57600',bytesize=8,parity='N',stopbits=1, timeout=3):
        self.prev_linear_velocity = 0
        self.prev_angular_velocity = 0
        self.connection = Serial(port=port, baudrate=baudrate, bytesize=bytesize, parity=parity,stopbits=stopbits, timeout=timeout)
        self.flush()


    def flush(self):
        self.connection.flushInput()
        self.connection.flushOutput()

    def open(self):
        if not self.connection.isOpen():    
            self.connection.open()
   
    def close(self):
        self.flush()
        self.connection.close()

    def get_float_from_hex(self, data):
        high_byte = data[0]
        low_byte = data[1]
        result = high_byte << 8 | low_byte
        result = c_short(result).value
        return result 

    def start_receive_data(self):
        read_thread = Thread(target=self.read_thread)
        read_thread.daemon = True
        read_thread.start()

    def get_serial_package(self): 
        #self.open()              
        if self.connection.readable:
            data = self.connection.read(1)
            #print ('--------------------------------------------')
            #print ('stx :', data)
            if data != b'' and data[0] == 2:
                buffer = data + self.connection.read(16)
                #print (buffer)
                return buffer      

    def read_thread(self):
        self.open()
        print ('Start thread of receiving data')

        while True:              
            self.data_buffer = self.get_serial_package()
            #print ('data buffer : ',self.data_buffer)
            if self.data_buffer != None:                
                try: 
                    if len(self.data_buffer) == 17:
                        if self.data_buffer[0] == 2 and self.data_buffer[15] == 3 and self.data_buffer[16] == 4: 

                            #print (self.data_buffer[1], self.data_buffer[2])
                            self.get_amv_status(self.data_buffer[1], self.data_buffer[2])  

                            if self.status_data.emergency_button == True:
                                self.status_data.linear_velocity = 0.0
                                self.status_data.angular_velocity = 0.0
                            else:
                                self.status_data.linear_velocity = self.get_float_from_hex(self.data_buffer[3:5])/1000 
                                self.status_data.angular_velocity = self.get_float_from_hex(self.data_buffer[5:7])/1000

                            self.status_data.battery_level = self.get_float_from_hex(self.data_buffer[7:9])
                            self.status_data.ultrasonic = self.get_float_from_hex(self.data_buffer[9:11])
                            self.status_data.system_fault_code = self.get_float_from_hex(self.data_buffer[11:13])
                            self.status_data.motor_fault_code = self.get_float_from_hex(self.data_buffer[13:15])       

                            if (self.status_data.linear_velocity - self.prev_linear_velocity) > 0.7:
                                self.status_data.linear_velocity = self.prev_linear_velocity
                                self.status_data.angular_velocity = self.prev_angular_velocity
                                #print('----------------------Shoot-------------------')
                                #print(self.status_data.linear_velocity - self.prev_linear_velocity)
                                print('cur')
                                print(self.status_data.linear_velocity)
                                print('prev')
                                print(self.prev_linear_velocity)
                            else:
                                self.prev_linear_velocity = self.status_data.linear_velocity
                                self.prev_angular_velocity = self.status_data.angular_velocity
                                print('-------------------Norm---------------------')
                                print(self.status_data.linear_velocity - self.prev_linear_velocity)
                                print('cur')
                                print(self.status_data.linear_velocity)
                                print('prev')
                                print(self.prev_linear_velocity)

                            
                            self.status_callback(self.status_data)                   
                except:
                    print ('Data reading wrong format')


    def get_amv_status(self, status_high, status_low):
        if (status_low & 1) != 0: 
            self.status_data.robot_ready = True 
        else: 
            self.status_data.robot_ready = False

        if (status_low & 2) != 0: 
            self.status_data.robot_moving = True
        else: 
            self.status_data.robot_moving = False

        if (status_low & 4) != 0: 
            self.status_data.stop_conf = True 
        else: 
            self.status_data.stop_conf = False

        if (status_low & 8) != 0: 
            self.status_data.auto_mode = True 
        else: 
            self.status_data.auto_mode = False

        if (status_low & 16) != 0: 
            self.status_data.linear_down_conf = True 
        else: 
            self.status_data.linear_down_conf = False

        if (status_low & 32) != 0: 
            self.status_data.linear_up_conf = True 
        else: 
            self.status_data.linear_up_conf = False

        if (status_low & 64) != 0: 
            self.status_data.green_button = True
        else: 
            self.status_data.green_button = False

        if (status_low & 128) != 0: 
            self.status_data.red_button = True
        else: 
            self.status_data.red_button = False

        if (status_high & 1) != 0: 
            self.status_data.system_fault = True 
        else: 
            self.status_data.system_fault = False

        if (status_high & 2) != 0: 
            self.status_data.emergency_button = True 
        else: 
            self.status_data.emergency_button = False 
        
        if (status_high & 64) != 0: 
            self.status_data.charging = True 
        else: 
            self.status_data.charging = False 
        
        if (status_high & 128) != 0: 
            self.status_data.initial_status = True 
        else: 
            self.status_data.initial_status = False 

    def status_callback(self, status):
        print (status)
        print ('----------------------------------------')

    def bytes_xor(self, var, key):
        return bytes(a ^ b for a, b in zip(var, key))

    def set_amv_velocity(self, lin_vel, ang_vel, amv_run, pin_lock, motor_lock, charger_on, led_command, buzzer_command):
        lin_vel = int(lin_vel)
        ang_vel = int(ang_vel) 
        lin_vel_byte = lin_vel.to_bytes(2,byteorder="big", signed=True)
        ang_vel_byte = ang_vel.to_bytes(2,byteorder="big", signed=True)

        command_high = b'\x00'
        command_low = b'\x00'

        if amv_run:
            command_low = self.bytes_xor(command_low, b'\x01')   #amv sart to run bit1

        if pin_lock:
            command_low = self.bytes_xor(command_low, b'\x08')   #linear up bit4
        else:
            command_low = self.bytes_xor(command_low, b'\x04')   #linear down bit3

        if motor_lock:
            command_low = self.bytes_xor(command_low, b'\x10')   #amv sart to run bit 5
        
        if charger_on:
            command_low = self.bytes_xor(command_low, b'\x20')   #auto charger on bit 6

        command = command_high + command_low + lin_vel_byte + ang_vel_byte + led_command + buzzer_command

        #loop for sumcheck
        sumcheck = b'\x00'
        for i in range(8):
            command_byte = command[i].to_bytes(1,byteorder="big")
            sumcheck = self.bytes_xor(sumcheck, command_byte)

        amv_command = b'\x02' + command + sumcheck + b'\x03\x04'
      
        self.send(amv_command)
        

    def move_froward(self):
        command = b'\x02\x00\x00\x00\x02\x00\x00\x03\x04'
        self.send(command)
    
    def move_stop(self):
        command = b'\x02\x00\x00\x00\x00\x00\x00\x03\x04'
        self.send(command)
   
    def send(self, command):
        self.open()
        if self.connection.writable:
            self.connection.write(command)

    def start_send_data(self):
        write_thread = Thread(target=self.write_thread)
        write_thread.daemon = True
        write_thread.start()

    def write_thread(self):
        self.open()
        print ('Start thread of sending data')
        while True:
            self.move_front()
            time.sleep(0.1)

if __name__ == "__main__":
    serial = BaseConnect()
    serial.start_receive_data()
    time.sleep(3)
    while True:        
        time.sleep(0.05)
        #serial.set_amv_velocity(-100,000,False)






