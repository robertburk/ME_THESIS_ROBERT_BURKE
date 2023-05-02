#!/usr/bin/env python3

'''
Author: Robert Burke <robertljburke@gmail.com>

Github: @robertburk

'''

import rospy
import numpy as np
from simple_pid import PID

from std_msgs.msg import Bool
from ucd_robot.msg import SystemState
from gripper.msg import TargetForce
from papillarray_ros_v2.msg import SensorState
from gripper.msg import TargetDelta

POSITION_CONTROLLER_NODE_RATE = 500.0

# Constants
DELTA_LIMIT = 100 # Estimated maximum movement per MX28 loop
OUTPUT_INTERVAL = (1.0/POSITION_CONTROLLER_NODE_RATE) # Min. time between new target forces
OPEN_LIMIT = -28000
CLOSE_LIMIT = 28000
# FORCE_ACCURACY = 0.1 # Accepted force accuracy: +/- 0.2N

PID_P = 8 #10
PID_I = 0 #0.5*PID_P #160 #
PID_D = 0 #0.5

class PIDController():
    '''
    This class subscribes to both sensors 
    '''

    def __init__(self):

        self.node_string = 'PID Node'
        rospy.init_node('Delta Position Controller', anonymous=True)
        rospy.loginfo('[%10s] Starting %s'%( self.node_string, self.__class__.__name__))

        # Initialise state variables
        self.system_state = False
        self.sensor_1_state = False
        self.sensor_2_state = False

        # Read Params
        self.DELTA_LIMIT = rospy.get_param('PID_DELTA_LIMIT')
        self.FORCE_ACCURACY = rospy.get_param('FORCE_ACCURACY')
        self.MAX_FORCE = rospy.get_param('MAX_FORCE')
        self.NODE_RATE = rospy.get_param('NODE_RATE')

        # Initialise Variables
        self.measured_fZ = np.array([0.0,0.0])
        self.target_force = 0
        self.error = 0
        self.pid = PID(PID_P, PID_I, PID_D,sample_time=0.025,output_limits=(-self.DELTA_LIMIT,self.DELTA_LIMIT),setpoint=0)
        self.last_reset_time = 0
        self.last_output_time = 0

        # Initiate Publishers
        self.pub_target_delta = rospy.Publisher('target_delta',TargetDelta,queue_size=1)

        # Initiate Subscribers
        self.sub_shutdown = rospy.Subscriber('shutdown',Bool,self.shutdown)
        self.sub_system_state = rospy.Subscriber('system_state',SystemState,self.read_system_state)
        self.sub_target_force = rospy.Subscriber('target_force',TargetForce,self.read_target_force)
        self.sub_sensor_state_1 = rospy.Subscriber('/hub_0/sensor_0',SensorState,self.read_sensor_1)
        self.sub_sensor_state_2 = rospy.Subscriber('/hub_0/sensor_1',SensorState,self.read_sensor_2)

        self.rate = rospy.Rate(self.NODE_RATE)
        self.rosrun()

    def read_system_state(self, msg:SystemState):
        if ( msg.system_state ) and ( not self.system_state ):
            rospy.loginfo('[%10s] System State: True'%self.node_string)
            self.system_state = True

        elif ( not msg.system_state ) and ( self.system_state ):
            rospy.loginfo('[%10s] System State: False'%self.node_string)
            self.system_state = False

    def read_sensor_1(self, msg:SensorState):
        if ( not self.sensor_1_state ):
            self.sensor_1_state = True

        self.measured_fZ[0] = msg.gfZ
        self.PID_controller()

    def read_sensor_2(self, msg:SensorState):
        if ( not self.sensor_2_state ):
            self.sensor_2_state = True

        self.measured_fZ[1] = msg.gfZ
        self.PID_controller()

    def read_target_force(self, msg:TargetForce):
        new_target = msg.target_force / 2.0
        if new_target != self.target_force:
            if new_target > self.MAX_FORCE:
                self.pid.setpoint = self.MAX_FORCE
                self.target_force = self.MAX_FORCE
            else:
                self.pid.setpoint = new_target
                self.target_force = new_target

    def PID_controller(self):
        '''
        This function calculates the error in measured force from the target force of the sensor under 
        the minimum force and drives this error towards zero
        '''
        if ( self.system_state ) and ( rospy.get_time() - self.last_output_time > OUTPUT_INTERVAL ):
            self.target_delta = self.pid(np.min(self.measured_fZ))
            self.pub_target_delta.publish(TargetDelta(self.target_delta)) 
            self.last_output_time = rospy.get_time()

    def shutdown(self, msg:Bool):
        if (msg.data):
            rospy.loginfo('[%10s] shutdown registered'%self.node_string )
            rospy.signal_shutdown('[%10s] shutdown registered'%self.node_string )

    def rosrun(self):
        while not rospy.is_shutdown():
            rospy.spin()

if __name__ == '__main__':
    try:
        hand = PIDController()

    except rospy.ROSInterruptException:
        pass