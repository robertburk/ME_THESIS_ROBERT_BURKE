#!/usr/bin/env python3


'''
Author: Robert Burke <robertljburke@gmail.com>

Github: @robertburk

This is the main class you can use for interfacing with the motor controller

'''

import rospy
import numpy as np

from std_msgs.msg import Bool
from ucd_robot.msg import SystemState
from MX28_node import MX28_Controller
from gripper.msg import TargetDelta, TargetForce
from papillarray_ros_v2.msg import SensorState
 
# Constants
FORCE_ACCURACY      = 0.2           # Accepted force accuracy: +/- 0.2N
WRITE_FREQUENCY     = 0.0025        # Write frequency of servo [ms]
READ_FREQUENCY      = 0.2           # Read frequency of servo [ms]

# Following values are taken from the MX-28 datasheet available at:  https://emanual.robotis.com/docs/en/dxl/mx/mx-28/
MAX_SERVO_TORQUE    = 1023          # Maximum value of MX28 torque 
SERVO_RESOLUTION    = 1
COM_PORT            = "/dev/ttyUSB0"
SERVO_ID            = 1


class Gripper_exp():

    '''
    This class interfaces between the ucd_robot_ws and the MX28 controller
    '''

    def __init__(self):
        self.node_string = 'Gripper'
        rospy.init_node('Gripper', anonymous=True)
        rospy.loginfo('[%10s] Starting %s'%( self.node_string, self.__class__.__name__))

        # Initialisers
        self.controller = MX28_Controller(port_num=COM_PORT, servo_id=SERVO_ID,resolution=SERVO_RESOLUTION,
                                          torque=MAX_SERVO_TORQUE,print_log=True)

        # Initiate system variables
        self.torque_enabled = False
        self.system_state = False
        self.move_state = False
        
        # Initiate gripper position variables
        self.target_delta = 0
        self.target_force = 0
        self.current_position = 0
        self.target_position = 0
        self.initial_position = 0
        self.prev_target_time = 0
        self.prev_position_time = 0
        self.measured_gfZ = np.array([0,0])
        self.critical_torque_limit = False
        self.max_force_limit = False

        # Read params
        self.MAX_FORCE = rospy.get_param('MAX_FORCE')
        self.CRITICAL_FORCE = rospy.get_param('CRITICAL_FORCE')
        self.node_rate = rospy.get_param('NODE_RATE')
        self.WRITE_FREQUENCY = 0.0025

        # Initiate publishers
        self.pub_gripper_state = rospy.Publisher('gripper_state',Bool,queue_size=1)

        # Initiate subscribers
        self.sub_shutdown = rospy.Subscriber('shutdown',Bool,self.shutdown)
        self.sub_system_state = rospy.Subscriber('system_state',SystemState,self.read_system_state)
        self.sub_target_force = rospy.Subscriber('target_force',TargetForce,self.read_target_force)
        self.sub_sensor_1_state = rospy.Subscriber('/hub_0/sensor_0',SensorState,self.read_sensor_1)
        self.sub_sensor_2_state = rospy.Subscriber('/hub_0/sensor_1',SensorState,self.read_sensor_2)
        self.sub_target_delta = rospy.Subscriber('target_delta',TargetDelta,self.read_target_delta)
        self.sub_gripper_move_state = rospy.Subscriber('gripper_move_state',Bool,self.read_move_state)

        # Read current position
        read_success = False
        while ( not read_success ):
            position = self.controller.MX28_pos()
            if ( type(position) != None ):
                self.initial_position = position
                self.current_position = position
                self.prev_position_time = rospy.get_time()
                read_success = True

        # Enable torque
        self.enable_torque()
        self.rate = rospy.Rate(self.node_rate)
        self.rosrun()

    def read_system_state(self,msg:SystemState):
        if ( msg.system_state ) and ( not self.system_state ) and ( not self.critical_torque_limit):
            rospy.loginfo('[%10s] System State: True'%self.node_string)
            self.move_state = True
            self.system_state = True

        elif ( not msg.system_state ) and ( self.system_state ):
            self.move_state = False
            rospy.loginfo('[%10s] Move State: False'%self.node_string)
            self.system_state = False

        if ( not msg.system_state ) and ( self.torque_enabled ) :
            self.pub_gripper_state.publish(Bool(True))

    def read_move_state(self,msg:Bool):
        if ( msg.data ) and ( self.move_state == False ):
            rospy.loginfo('[%10s] Move Enabled'%self.node_string)
            self.move_state = True
        if ( not msg.data ) and ( self.move_state == True):
            rospy.loginfo('[%10s] Move Disabled'%self.node_string)
            self.move_state = False

    def enable_torque(self):
        if ( not self.torque_enabled ):
            torque_enabled = False
            while not torque_enabled:
                torque_enabled = self.controller.MX28_start()
            self.pub_gripper_state.publish(Bool(True))
            rospy.loginfo('[%10s] Torque Enabled'%self.node_string)
            self.torque_enabled = True
           
    def read_target_force(self, msg:TargetForce):
        self.target_force = msg.target_force
  
    def read_sensor_1(self, msg:SensorState): 
        self.measured_gfZ[0] = float(msg.gfZ)
        self.critical_force()
    
    def read_sensor_2(self, msg:SensorState): 
        self.measured_gfZ[1] = msg.gfZ
        self.critical_force()

    def read_target_delta(self, msg:TargetDelta):
        self.target_delta = msg.target_delta
        self.write_target_position()

    def write_target_position(self):
        if ( rospy.get_time() - self.prev_target_time > WRITE_FREQUENCY ) and ( self.move_state ):

            # Read position

            if ( rospy.get_time() - self.prev_position_time > READ_FREQUENCY ):

                read_success = False
                while ( not read_success ):
                    position = self.controller.MX28_pos()
                    if ( type(position) != None ):
                        self.current_position = position
                        read_success = True
                self.prev_position_time = rospy.get_time()

            # Calculate target position

            if ( self.move_state == True ) : 
                if ( self.target_force == 0.0 ):
                    self.target_position = self.initial_position 
                elif ( not self.max_force_limit):
                    self.target_position = int(self.current_position) + self.target_delta                    
                elif ( self.target_delta <= 0):
                    self.target_position = int(self.current_position) + self.target_delta

            else:
                self.target_position = self.current_position

            # If the target position is new, write it

            if ( self.target_position != self.current_position ):

                # write_success = False
                # while ( not write_success ):
                write_success = self.controller.MX28_target(int(self.target_position))
                self.current_position = self.target_position
            
            self.prev_target_time = rospy.get_time()

    def critical_force(self):
        # If at critical force, toggle the critical force flag, disable torque
        if ( self.measured_gfZ[0] > self.CRITICAL_FORCE ) or (self.measured_gfZ[1] > self.CRITICAL_FORCE):
            self.move_state = False
            self.critical_torque_limit = True
            self.controller.MX28_stop()
            rospy.loginfo('[%10s] Critical force measured, move disabled'%self.node_string)
            self.shutdown(Bool(True))
        
        # If at max force, toggle the max force flag
        elif  (self.measured_gfZ[0] > self.MAX_FORCE ) or (self.measured_gfZ[1] > self.MAX_FORCE):
            self.max_force_limit = True

        # If below max force again, toggle the flag
        elif (self.measured_gfZ[0] < self.MAX_FORCE ) and (self.measured_gfZ[1] < self.MAX_FORCE) and (self.max_force_limit):
            self.max_force_limit = False

    def shutdown(self, msg:Bool):
        if (msg.data) and ( not self.critical_torque_limit):
            self.move_state = False
            write_success = False
            while ( not write_success ):
                write_success = self.controller.MX28_stop()
        rospy.loginfo('[%10s] shutdown registered'%self.node_string)
        rospy.signal_shutdown('[%10s] shutdown registered'%self.node_string)

    def rosrun(self):
        while not rospy.is_shutdown():
            rospy.spin()
            # self.write_target_position()


if __name__ == '__main__':
    try:
        hand = Gripper_exp()

    except rospy.ROSInterruptException:
        pass