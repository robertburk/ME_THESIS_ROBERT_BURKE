#!/usr/bin/env python
'''
Author: Robert Burke <robertljburke@gmail.com>

Github: @robertburk

This is the main class you can use for controlling the gripper

'''

from Robotic_Servos import openport,openpacket,Robotis_Servo


class MX28_Controller():


    def __init__(self, port_num, servo_id,torque,resolution=1,print_log=False):
        '''
        port_num: port on your PC that connect with the servo, you can check it from system's device manager
        id: servo's ID, you can check it from dymanixel wizard
        '''
        self.id = servo_id
        self.port_num = port_num
        self.port = openport(port_num)
        self.packet = openpacket()
        self.servo = Robotis_Servo(self.port, self.packet, self.id)
        self.servo.init_multiturn_mode()
        self.servo.set_resolution(resolution)
        self.servo.set_return_delay_time(0)
        self.print_log = print_log
        self.mode = 'multiturn'

        # Set the open and close limits        
        self.close_limit = int(22000)
        self.open_limit = int(-15000)
        
    def MX28_target(self, target):
        success = self.servo.moveit(target, self.mode)
        if self.print_log and success:
            return True
            
    def MX28_close(self,speed=100):
        if speed == 0:
            self.MX28_stop()
            print("Gripper is stopped")
        else:
            self.servo.moveit(self.close_limit)
            if self.print_log:
                print("Gripper is closing")

    def MX28_open(self,speed=100):
        if speed == 0:
            self.MX28_stop()
            print("Gripper is stopped")
        else:
            self.servo.moveit(self.open_limit)
            if self.print_log:
                print("Gripper is opening")
      
    def MX28_speed(self):
        return self.servo.read_speed()
    
    def MX28_load(self):
        return self.servo.read_load()

    def MX28_goal(self):
        return self.servo.read_goal()
    
    def MX28_pos(self):
        return self.servo.read_current_pos()

    def MX28_stop(self): 
        self.servo.disable_torque()
        
    def MX28_start(self): 
        self.servo.enable_torque()
        return True
        
    
        

