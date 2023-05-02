#!/usr/bin/env python3

'''
Author: Robert Burke <robertljburke@gmail.com>

Github: @robertburk

'''

import rospy
import numpy as np
import pandas as pd
import random as rand

from std_msgs.msg import Bool, String
from papillarray_ros_v2.msg import SensorState
from papillarray_ros_v2.srv import BiasRequest, StartSlipDetection, StopSlipDetection
from ucd_robot.msg import SystemState
from gripper.msg import TargetForce
from tf2_msgs.msg import TFMessage
from geometry_msgs.msg import Pose, Quaternion, Vector3

UCD_ROBOT_RATE = 500
BUFFER_LENGTH = 500
TEST_LENGTH_TIME = 60 # [seconds]
SENSOR_DELAY_TIME = 0.5
TEST_TIME = 5 # hold each position for 5 seconds
NUMBER_TESTS = 1
MIN_GRIP_FORCE = 2
MAX_GRIP_FORCE = 18
FORCE_ACCURACY = 0.2

class UCDRobot_react():

    '''
    This class implements the experimental procedure for the reactive protocol
    '''

    def __init__(self):
        
        self.node_string = 'UCD Robot'
        rospy.init_node('UCD Robot', anonymous=True)
        rospy.loginfo('[%10s] Starting %s'%( self.node_string, self.__class__.__name__))

        # Initiate hardware flags
        self.sensor_connect = [False, False]
        self.bias_state = False
        self.buffer_state = [False, False]
        self.sensor_state = [False, False]
        self.sensors_hub_state = False
        self.robot_state = False
        self.gripper_state = False
        self.camera_state = False
        self.system_state = False
        self.experiment_complete = False
        self.start_experiment = False

        # Initiate Variables
        self.i = [0, 0]
        self.measured_gfZ = np.array([0.0, 0.0])
        self.avg_measured_gfZ = 0
        self.prev_sensor_read = np.array([0, 0])
        self.init_z = -1
        self.z = 0

        # Read constants
        self.node_rate = rospy.get_param('NODE_RATE')
        self.GRIP_FORCE = rospy.get_param('GRIP_FORCE')
        self.EXP_NUM = rospy.get_param('EXP_NUM')

        # Read paths
        self.path = rospy.get_param('/save_path')
        self.object_type = rospy.get_param('/object_type')
        self.object_weight = rospy.get_param('/object_weight')
        self.object_distance = rospy.get_param('/object_distance')
        self.save_path = str(self.path) + '/' + str(self.EXP_NUM)  + '_' + str(self.GRIP_FORCE) + '_' + str(self.object_weight) + '_' + str(self.object_distance)

        # Initiate variables for data export
        self.sensor_1_data = {}
        self.sensor_2_data = {}
        self.target_force_data = {}

        # Initiate publishers
        self.pub_shutdown = rospy.Publisher('shutdown',Bool,queue_size=1)
        self.pub_system_state = rospy.Publisher('system_state',SystemState,queue_size=1)
        self.pub_gripper_move_state = rospy.Publisher('gripper_move_state',Bool,queue_size=1)
        self.pub_export_images = rospy.Publisher('export_images',String,queue_size=1)

        # Initiate controlling publishers
        self.pub_target_force = rospy.Publisher('target_force',TargetForce, queue_size=1)
        self.pub_target_pose = rospy.Publisher('target_pose',Pose, queue_size=1)

        # Initiate subscribers
        self.sub_sensor_1_state = rospy.Subscriber('/hub_0/sensor_0',SensorState,self.read_sensor_1)
        self.sub_sensor_2_state = rospy.Subscriber('/hub_0/sensor_1',SensorState,self.read_sensor_2)
        self.sub_robot_state = rospy.Subscriber('robot_state',Bool,self.read_robot_state)
        self.sub_gripper_state = rospy.Subscriber('gripper_state',Bool,self.read_gripper_state)
        self.sub_target_force = rospy.Subscriber('target_force',TargetForce,self.read_target_force)
        self.sub_camera_state = rospy.Subscriber('camera_state',Bool,self.read_camera_state)
        self.sub_program_running = rospy.Subscriber('/ur_hardware_interface/robot_program_running',Bool,self.read_robot_state)
        self.sub_joint_state = rospy.Subscriber('tf',TFMessage, self.read_joint_state)

        self.rate = rospy.Rate(self.node_rate)
        self.rosrun()

    def request_bias(self):
        rospy.wait_for_service("/hub_0/send_bias_request")
        bias_serv = rospy.ServiceProxy("/hub_0/send_bias_request",BiasRequest)
        resp = bias_serv.call()
        if resp.result:
            self.bias_state = True
            rospy.loginfo('[%10s] System Check: bias state %s'%( self.node_string, self.bias_state))
        else:
            raise RuntimeError('Bias sensor error')

    def start_slip_detection(self):
        rospy.wait_for_service("/hub_0/start_slip_detection")
        slip_serv = rospy.ServiceProxy("/hub_0/start_slip_detection",StartSlipDetection)
        resp = slip_serv.call()
        if resp.result:
            rospy.loginfo('[%10s] System Check: slip detection started'%( self.node_string))
        else:
            raise RuntimeError('Slip detection error')
        
    def system_check(self):

        system_state = SystemState()
        system_state.robot_state = self.robot_state
        system_state.gripper_state = self.gripper_state
        if (self.sensor_state[0] == self.sensor_state[1] ) and ( self.sensor_state[0] == True):
            system_state.sensor_state = self.sensor_state[0]
        system_state.camera_state = self.camera_state

        if ( self.sensor_state[0] ) and ( self.sensor_state[1] ) and ( self.robot_state ) and ( self.gripper_state ) and ( self.camera_state ) and ( not self.system_state ):
            rospy.loginfo('[%10s] System State: True'%(self.node_string))
            user_choice = 0 # 0 = ask, 1 = continue, -1 = stop
            # continue_test
            while user_choice == 0:
                input_str = input('[%10s] Start protocol? (1 = Yes, 0 = Stop)  \n'%(self.node_string))
                try:
                    choice = int(input_str)
                    if ( choice == 1):
                        rospy.loginfo('[%10s] User input: 1, continuing test'%self.node_string)
                        self.pub_gripper_move_state.publish(Bool(True))
                        user_choice = 1
                    elif ( choice == 0 ):
                        rospy.loginfo('[%10s] User input: 0, stopping test'%self.node_string)
                        self.pub_shutdown.publish(Bool(True))
                    else:
                        rospy.loginfo('[%10s] User input invalid'%self.node_string)
                except ValueError:
                    rospy.loginfo("Input is not a valid number. Please try again.")
            if user_choice == 1:            
                system_state.system_state = True
                self.system_state = True

        elif (not self.sensor_state[0] ) or ( not self.sensor_state[1])  or ( not self.robot_state ) or ( not self.gripper_state ) or ( not self.camera_state ):
            if ( self.system_state ) :
                if ( not self.sensor_state[0] ) or ( not self.sensor_state[1]):
                    rospy.loginfo('[%10s] Sensor State: %s'%(self.node_string,self.sensor_state))
                    self.system_state = False
                if ( not self.robot_state ):
                    rospy.loginfo('[%10s Robot State: %s'%(self.node_string,self.robot_state))
                    self.system_state = False
                if ( not self.gripper_state ):
                    rospy.loginfo('[%10s] Gripper State: %s'%(self.node_string,self.gripper_state))
                    self.system_state = False
                if ( not self.camera_state ):
                    rospy.loginfo('[%10s] Camera State: %s'%(self.node_string,self.camera_state))
                    self.system_state = False

            system_state.system_state = self.system_state
            self.pub_system_state.publish(system_state)

        else: 
            system_state.system_state = self.system_state

        self.pub_system_state.publish(system_state)

    def read_target_force(self, msg:TargetForce):
        if ( self.system_state ) and ( not self.experiment_complete):
            self.target_force = msg.target_force
            t = rospy.get_time()
            self.target_force_data[f'{t}'] = self.target_force

    def read_robot_state(self, msg:Bool):
        if ( msg.data ) and ( not self.robot_state ):
            rospy.loginfo('[%10s] Robot Connected'%self.node_string)
            self.robot_state = True
        elif ( not msg.data ) and ( self.robot_state ):
            rospy.loginfo('[%10s] Robot Disconnected'%self.node_string)       
            self.robot_state = False

    def read_gripper_state(self, msg:Bool):
        if ( msg.data ) and ( not self.gripper_state ):
            rospy.loginfo('[%10s] Gripper Connected'%self.node_string)
            self.gripper_state = True
        elif ( not msg.data ) and ( self.gripper_state ):
            rospy.loginfo('[%10s] Gripper Disconnected'%self.node_string)
            self.gripper_state = False

    def read_sensor_1(self, msg:SensorState):

        if ( not self.sensor_connect[0] ):
            rospy.loginfo('[%10s] Sensor 1 Connected'%self.node_string)
            self.request_bias()
            self.sensor_connect[0] = True

        if ( self.bias_state ) :
            self.i[0] += 1

        if ( self.i[0] > BUFFER_LENGTH ) and ( not self.buffer_state[0] ):
            rospy.loginfo('[%10s] Buffer 1 State: true'%self.node_string)
            self.buffer_state[0] = True
        
        if ( self.buffer_state[0] ) and ( self.bias_state ) and ( not self.sensor_state[0] ):
            rospy.loginfo('[%10s] Sensor 1 Ready'%self.node_string)
            self.sensor_state[0] = True

        if ( self.system_state ) and ( not self.experiment_complete):
            self.measured_gfZ[0] = msg.gfZ
            self.avg_measured_gfZ = np.mean(self.measured_gfZ)
            t = rospy.get_time()
            self.sensor_1_data[f'{t}'] = msg
        
        self.prev_sensor_read[0] = rospy.get_time()

    def read_sensor_2(self, msg:SensorState):

        if ( not self.sensor_connect[1] ):
            rospy.loginfo('[%10s] Sensor 2 Connected'%self.node_string)
            self.request_bias()
            self.sensor_connect[1] = True

        if ( self.bias_state ) :
            self.i[1] += 1

        if ( self.i[1] > BUFFER_LENGTH ) and ( not self.buffer_state[1] ):
            rospy.loginfo('[%10s] Buffer 2 State: true'%self.node_string)
            self.buffer_state[1] = True
        
        if ( self.buffer_state[1] ) and ( self.bias_state ) and ( not self.sensor_state[1] ):
            rospy.loginfo('[%10s] Sensor 2 Ready'%self.node_string)
            self.sensor_state[1] = True

        if ( self.system_state ) and ( not self.experiment_complete):
            self.measured_gfZ[1] = msg.gfZ
            self.avg_measured_gfZ = np.mean(self.measured_gfZ) 
            t = rospy.get_time()
            self.sensor_2_data[f'{t}'] = msg
        
        self.prev_sensor_read[1] = rospy.get_time()

    def read_camera_state(self,msg:Bool):
        if ( msg.data ) and ( not self.camera_state ):
            rospy.loginfo('[%10s] Camera Connected'%self.node_string)
            self.camera_state = True

        elif ( not msg.data ) and ( self.camera_state ):
            rospy.loginfo('[%10s] Camera Disconnected'%(self.node_string))
            self.camera_state = False

    def read_joint_state(self, msg:TFMessage):
        if ( self.init_z == -1 ) :
            if (msg.transforms[0].transform.translation.z != 0 ) :
                self.init_z = msg.transforms[0].transform.translation.z
        
        if (msg.transforms[0].transform.translation.z != 0 ) :
            self.z = msg.transforms[0].transform.translation.z

    def experimental_procedure(self):

        self.exp_string = 'EXP PRTCL'

        # rospy.loginfo('[%10s] Experiment Begun'%self.exp_string)
        self.experiment_start_time = rospy.get_time()
        rospy.loginfo('[%10s] Beginning reactive protocol '%self.exp_string)

        # ---------------------         Grip Object         ----------------------#

        rospy.sleep(3.0)

        # ---------------------         Raise Robot         ----------------------#


        rospy.loginfo('[%10s] Instructing robot to raise'%self.exp_string)
        up_pose = Pose(
            Vector3(0, 0, 0.04), Quaternion(0, 0, 0, 0)
        )
        self.pub_target_pose.publish(up_pose)
        
        rospy.loginfo('[%10s] Robot raised'%(self.exp_string))
        
        self.test_start_time = rospy.get_time()


        # ---------------------         Hold for 20 sec     ----------------------#


        while ( self.target_force != 0.0 ):
            rospy.sleep(0.50)


        # ----------------------        Open gripper        ----------------------#

        self.target_force = 0.0

        target_force = TargetForce()
        target_force.target_force = self.target_force
        self.pub_target_force.publish(target_force)


        rospy.loginfo('[%10s] Opening gripper'%self.exp_string)


        #----------------------      Ask User before lowering----------------------#


        user_choice = 0 # 0 = ask, 1 = continue, -1 = stop
        # continue_test
        while user_choice == 0:
            input_str = input('[%10s] Safe to lower robot? (1 = Yes, 0 = Stop)  \n'%(self.exp_string))
            try:
                choice = int(input_str)
                if ( choice == 1):
                    rospy.loginfo('[%10s] User input: 1, continuing test'%self.exp_string)
                    user_choice = 1
                elif ( choice == 0 ):
                    rospy.loginfo('[%10s] User input: 0, stopping test'%self.exp_string)
                    # user_choice = -1
                else:
                    rospy.loginfo('[%10s] User input invalid'%self.exp_string)
            except ValueError:
                rospy.loginfo("Input is not a valid number. Please try again.")


        #----------------------         Lower Robot         ----------------------#


        rospy.loginfo('[%10s] instructing robot to lower'%(self.exp_string))

        # Move robot
        down_pose = Pose(
            Vector3(0, 0, -0.04), Quaternion(0, 0, 0, 0)
        )
        self.pub_target_pose.publish(down_pose)
        rospy.sleep(1.0)

        rospy.loginfo('[%10s] robot lowered'%(self.exp_string))


        #----------------------   Ask User before saving     ----------------------#


        user_choice = 0 # 0 = ask, 1 = continue, -1 = stop
        # continue_test
        while user_choice == 0:
            input_str = input('[%10s] Save files? (1 = Yes, 0 = Stop)  \n'%(self.exp_string))
            try:
                choice = int(input_str)
                if ( choice == 1):
                    rospy.loginfo('[%10s] User input: 1, continuing test'%self.exp_string)
                    user_choice = 1
                elif ( choice == 0 ):
                    rospy.loginfo('[%10s] User input: 0, stopping test'%self.exp_string)
                    self.pub_shutdown.publish(Bool(True))
                    # user_choice = -1
                else:
                    rospy.loginfo('[%10s] User input invalid'%self.exp_string)
            except ValueError:
                rospy.loginfo("Input is not a valid number. Please try again.")

        rospy.loginfo('[%10s] Test complete'%(self.exp_string))
        export_string = self.exp_string
        self.pub_export_images.publish(export_string)

        self.system_state = False

        save_sensor_data = False
        while not save_sensor_data:
            save_sensor_data = self.save_data()

        self.pub_shutdown.publish(Bool(True))

        rospy.sleep(3.)

        rospy.loginfo('[%10s] Experiment Complete'%self.exp_string)
        
        return True    
    
    def save_data(self):
        save_path_1 = self.save_path + '_sensor1.npy'
        save_path_2 = self.save_path + '_sensor2.npy'
        save_path_tf = self.save_path + '_tf.npy'
        np.save(save_path_1, self.sensor_1_data)
        np.save(save_path_2, self.sensor_2_data)
        np.save(save_path_tf, self.target_force_data)
        rospy.loginfo('[%10s] Save complete at:  %s, %s, and %s'% ( self.node_string, save_path_1, save_path_2, save_path_tf))
        return True

    def shutdown(self, msg:Bool):
        if (msg.data):
            rospy.loginfo('[%10s] shutdown registered'%self.node_string )
            rospy.signal_shutdown('[%10s] shutdown registered'%self.node_string )

    def rosrun(self):
        while ( self.system_state == False ):
            self.system_check()
        
        self.experimental_procedure()

class UCDRobot_const():

    '''
    This class implements the experimental procedure for the fixed grip force protocol
    '''

    def __init__(self):
        
        self.node_string = 'UCD Robot'
        rospy.init_node('UCD Robot', anonymous=True)
        rospy.loginfo('[%10s] Starting %s'%( self.node_string, self.__class__.__name__))

        # Initiate hardware flags
        self.sensor_connect = [False, False]
        self.bias_state = False
        self.buffer_state = [False, False]
        self.sensor_state = [False, False]
        self.sensors_hub_state = False
        self.robot_state = False
        self.gripper_state = False
        self.camera_state = False
        self.system_state = False
        self.experiment_complete = False
        self.start_experiment = False

        # Initiate Variables
        self.i = [0, 0]
        self.measured_gfZ = np.array([0.0, 0.0])
        self.avg_measured_gfZ = 0
        self.prev_sensor_read = np.array([0, 0])
        self.init_z = -1
        self.z = 0

        # Read constants
        self.node_rate = rospy.get_param('NODE_RATE')
        self.GRIP_FORCE = rospy.get_param('GRIP_FORCE')
        self.EXP_NUM = rospy.get_param('EXP_NUM')

        # Read paths
        self.path = rospy.get_param('/save_path')
        self.object_type = rospy.get_param('/object_type')
        self.object_weight = rospy.get_param('/object_weight')
        self.object_distance = rospy.get_param('/object_distance')
        self.save_path = str(self.path) + '/' + str(self.EXP_NUM) + '_' + str(self.GRIP_FORCE) + '_' + str(self.object_weight) + '_' + str(self.object_distance)

        # Initiate variables for data export
        self.sensor_1_data = {}
        self.sensor_2_data = {}
        self.target_force_data = {}

        # Initiate publishers
        self.pub_shutdown = rospy.Publisher('shutdown',Bool,queue_size=1)
        self.pub_system_state = rospy.Publisher('system_state',SystemState,queue_size=1)
        self.pub_gripper_move_state = rospy.Publisher('gripper_move_state',Bool,queue_size=1)
        self.pub_export_images = rospy.Publisher('export_images',String,queue_size=1)

        # Initiate controlling publishers
        self.pub_target_force = rospy.Publisher('target_force',TargetForce, queue_size=1)
        self.pub_target_pose = rospy.Publisher('target_pose',Pose, queue_size=1)

        # Initiate subscribers
        self.sub_sensor_1_state = rospy.Subscriber('/hub_0/sensor_0',SensorState,self.read_sensor_1)
        self.sub_sensor_2_state = rospy.Subscriber('/hub_0/sensor_1',SensorState,self.read_sensor_2)
        self.sub_robot_state = rospy.Subscriber('robot_state',Bool,self.read_robot_state)
        self.sub_gripper_state = rospy.Subscriber('gripper_state',Bool,self.read_gripper_state)
        self.sub_target_force = rospy.Subscriber('target_force',TargetForce,self.read_target_force)
        self.sub_camera_state = rospy.Subscriber('camera_state',Bool,self.read_camera_state)
        self.sub_program_running = rospy.Subscriber('/ur_hardware_interface/robot_program_running',Bool,self.read_robot_state)
        self.sub_joint_state = rospy.Subscriber('tf',TFMessage, self.read_joint_state)

        self.rate = rospy.Rate(self.node_rate)
        self.rosrun()

    def request_bias(self):
        rospy.wait_for_service("/hub_0/send_bias_request")
        bias_serv = rospy.ServiceProxy("/hub_0/send_bias_request",BiasRequest)
        resp = bias_serv.call()
        if resp.result:
            self.bias_state = True
            rospy.loginfo('[%10s] System Check: bias state %s'%( self.node_string, self.bias_state))
        else:
            raise RuntimeError('Bias sensor error')

    def start_slip_detection(self):
        rospy.wait_for_service("/hub_0/start_slip_detection")
        slip_serv = rospy.ServiceProxy("/hub_0/start_slip_detection",StartSlipDetection)
        resp = slip_serv.call()
        if resp.result:
            rospy.loginfo('[%10s] System Check: slip detection started'%( self.node_string))
        else:
            raise RuntimeError('Slip detection error')
        
    def system_check(self):

        system_state = SystemState()
        system_state.robot_state = self.robot_state
        system_state.gripper_state = self.gripper_state
        if (self.sensor_state[0] == self.sensor_state[1] ) and ( self.sensor_state[0] == True):
            system_state.sensor_state = self.sensor_state[0]
        system_state.camera_state = self.camera_state

        if ( self.sensor_state[0] ) and ( self.sensor_state[1] ) and ( self.robot_state ) and ( self.gripper_state ) and ( self.camera_state ) and ( not self.system_state ):
            rospy.loginfo('[%10s] System State: True'%(self.node_string))
            user_choice = 0 # 0 = ask, 1 = continue, -1 = stop
            # continue_test
            while user_choice == 0:
                input_str = input('[%10s] Start protocol? (1 = Yes, 0 = Stop)  \n'%(self.node_string))
                try:
                    choice = int(input_str)
                    if ( choice == 1):
                        rospy.loginfo('[%10s] User input: 1, continuing test'%self.node_string)
                        self.pub_gripper_move_state.publish(Bool(True))
                        user_choice = 1
                    elif ( choice == 0 ):
                        rospy.loginfo('[%10s] User input: 0, stopping test'%self.node_string)
                        self.pub_shutdown.publish(Bool(True))
                    else:
                        rospy.loginfo('[%10s] User input invalid'%self.node_string)
                except ValueError:
                    rospy.loginfo("Input is not a valid number. Please try again.")
            if user_choice == 1:            
                system_state.system_state = True
                self.system_state = True

        elif (not self.sensor_state[0] ) or ( not self.sensor_state[1])  or ( not self.robot_state ) or ( not self.gripper_state ) or ( not self.camera_state ):
            if ( self.system_state ) :
                if ( not self.sensor_state[0] ) or ( not self.sensor_state[1]):
                    rospy.loginfo('[%10s] Sensor State: %s'%(self.node_string,self.sensor_state))
                    self.system_state = False
                if ( not self.robot_state ):
                    rospy.loginfo('[%10s Robot State: %s'%(self.node_string,self.robot_state))
                    self.system_state = False
                if ( not self.gripper_state ):
                    rospy.loginfo('[%10s] Gripper State: %s'%(self.node_string,self.gripper_state))
                    self.system_state = False
                if ( not self.camera_state ):
                    rospy.loginfo('[%10s] Camera State: %s'%(self.node_string,self.camera_state))
                    self.system_state = False

            system_state.system_state = self.system_state
            self.pub_system_state.publish(system_state)

        else: 
            system_state.system_state = self.system_state

        self.pub_system_state.publish(system_state)

    def read_target_force(self, msg:TargetForce):
        if ( self.system_state ) and ( not self.experiment_complete):
            self.target_force = msg.target_force
            t = rospy.get_time()
            self.target_force_data[f'{t}'] = self.target_force

    def read_robot_state(self, msg:Bool):
        if ( msg.data ) and ( not self.robot_state ):
            rospy.loginfo('[%10s] Robot Connected'%self.node_string)
            self.robot_state = True
        elif ( not msg.data ) and ( self.robot_state ):
            rospy.loginfo('[%10s] Robot Disconnected'%self.node_string)       
            self.robot_state = False

    def read_gripper_state(self, msg:Bool):
        if ( msg.data ) and ( not self.gripper_state ):
            rospy.loginfo('[%10s] Gripper Connected'%self.node_string)
            self.gripper_state = True
        elif ( not msg.data ) and ( self.gripper_state ):
            rospy.loginfo('[%10s] Gripper Disconnected'%self.node_string)
            self.gripper_state = False

    def read_sensor_1(self, msg:SensorState):

        if ( not self.sensor_connect[0] ):
            rospy.loginfo('[%10s] Sensor 1 Connected'%self.node_string)
            self.request_bias()
            self.sensor_connect[0] = True

        if ( self.bias_state ) :
            self.i[0] += 1

        if ( self.i[0] > BUFFER_LENGTH ) and ( not self.buffer_state[0] ):
            rospy.loginfo('[%10s] Buffer 1 State: true'%self.node_string)
            self.buffer_state[0] = True
        
        if ( self.buffer_state[0] ) and ( self.bias_state ) and ( not self.sensor_state[0] ):
            rospy.loginfo('[%10s] Sensor 1 Ready'%self.node_string)
            self.sensor_state[0] = True

        if ( self.system_state ) and ( not self.experiment_complete):
            self.measured_gfZ[0] = msg.gfZ
            self.avg_measured_gfZ = np.mean(self.measured_gfZ) #(self.measured_gfZ[0] + self.measured_gfZ[1])/2.0
            t = rospy.get_time()
            self.sensor_1_data[f'{t}'] = msg
        
        self.prev_sensor_read[0] = rospy.get_time()

    def read_sensor_2(self, msg:SensorState):

        if ( not self.sensor_connect[1] ):
            rospy.loginfo('[%10s] Sensor 2 Connected'%self.node_string)
            self.request_bias()
            self.sensor_connect[1] = True

        if ( self.bias_state ) :
            self.i[1] += 1

        if ( self.i[1] > BUFFER_LENGTH ) and ( not self.buffer_state[1] ):
            rospy.loginfo('[%10s] Buffer 2 State: true'%self.node_string)
            self.buffer_state[1] = True
        
        if ( self.buffer_state[1] ) and ( self.bias_state ) and ( not self.sensor_state[1] ):
            rospy.loginfo('[%10s] Sensor 2 Ready'%self.node_string)
            self.sensor_state[1] = True

        if ( self.system_state ) and ( not self.experiment_complete):
            self.measured_gfZ[1] = msg.gfZ
            self.avg_measured_gfZ = np.mean(self.measured_gfZ) #(self.measured_gfZ[0] + self.measured_gfZ[1])/2.0
            t = rospy.get_time()
            self.sensor_2_data[f'{t}'] = msg
        
        self.prev_sensor_read[1] = rospy.get_time()

    def read_camera_state(self,msg:Bool):
        if ( msg.data ) and ( not self.camera_state ):
            rospy.loginfo('[%10s] Camera Connected'%self.node_string)
            self.camera_state = True

        elif ( not msg.data ) and ( self.camera_state ):
            rospy.loginfo('[%10s] Camera Disconnected'%(self.node_string))
            self.camera_state = False

    def read_joint_state(self, msg:TFMessage):
        if ( self.init_z == -1 ) :
            if (msg.transforms[0].transform.translation.z != 0 ) :
                self.init_z = msg.transforms[0].transform.translation.z
        
        if (msg.transforms[0].transform.translation.z != 0 ) :
            self.z = msg.transforms[0].transform.translation.z

    def experimental_procedure(self):

        self.exp_string = 'EXP PRTCL'

        # rospy.loginfo('[%10s] Experiment Begun'%self.exp_string)
        self.experiment_start_time = rospy.get_time()
        rospy.loginfo('[%10s] Beginning constant gf protocol '%self.exp_string)

        # ---------------------         Grip Object         ----------------------#

        rospy.sleep(3.0)

        # ---------------------         Raise Robot         ----------------------#


        rospy.loginfo('[%10s] Instructing robot to raise'%self.exp_string)
        up_pose = Pose(
            Vector3(0, 0, 0.04), Quaternion(0, 0, 0, 0)
        )
        self.pub_target_pose.publish(up_pose)
        # self.start_slip_detection()
        
        rospy.loginfo('[%10s] Robot raised'%(self.exp_string))
        
        self.test_start_time = rospy.get_time()


        # ---------------------         Hold for 20 sec     ----------------------#


        while ( self.target_force != 0.0 ):
            rospy.sleep(1.0)


        # ----------------------        Open gripper        ----------------------#

        self.target_force = 0.0

        target_force = TargetForce()
        target_force.target_force = self.target_force
        self.pub_target_force.publish(target_force)


        rospy.loginfo('[%10s] Opening gripper'%self.exp_string)


        #----------------------      Ask User before lowering----------------------#


        user_choice = 0 # 0 = ask, 1 = continue, -1 = stop
        # continue_test
        while user_choice == 0:
            input_str = input('[%10s] Safe to lower robot? (1 = Yes, 0 = Stop)  \n'%(self.exp_string))
            try:
                choice = int(input_str)
                if ( choice == 1):
                    rospy.loginfo('[%10s] User input: 1, continuing test'%self.exp_string)
                    user_choice = 1
                elif ( choice == 0 ):
                    rospy.loginfo('[%10s] User input: 0, stopping test'%self.exp_string)
                    # user_choice = -1
                else:
                    rospy.loginfo('[%10s] User input invalid'%self.exp_string)
            except ValueError:
                rospy.loginfo("Input is not a valid number. Please try again.")


        #----------------------         Lower Robot         ----------------------#


        rospy.loginfo('[%10s] instructing robot to lower'%(self.exp_string))

        # Move robot
        down_pose = Pose(
            Vector3(0, 0, -0.04), Quaternion(0, 0, 0, 0)
        )
        self.pub_target_pose.publish(down_pose)
        rospy.sleep(1.0)

        rospy.loginfo('[%10s] robot lowered'%(self.exp_string))


        #----------------------   Ask User before saving     ----------------------#


        user_choice = 0 # 0 = ask, 1 = continue, -1 = stop
        # continue_test
        while user_choice == 0:
            input_str = input('[%10s] Save files? (1 = Yes, 0 = Stop)  \n'%(self.exp_string))
            try:
                choice = int(input_str)
                if ( choice == 1):
                    rospy.loginfo('[%10s] User input: 1, continuing test'%self.exp_string)
                    user_choice = 1
                elif ( choice == 0 ):
                    rospy.loginfo('[%10s] User input: 0, stopping test'%self.exp_string)
                    self.pub_shutdown.publish(Bool(True))
                    # user_choice = -1
                else:
                    rospy.loginfo('[%10s] User input invalid'%self.exp_string)
            except ValueError:
                rospy.loginfo("Input is not a valid number. Please try again.")

        rospy.loginfo('[%10s] Test complete'%(self.exp_string))
        export_string = self.exp_string
        self.pub_export_images.publish(export_string)

        self.system_state = False

        save_sensor_data = False
        while not save_sensor_data:
            save_sensor_data = self.save_data()

        self.pub_shutdown.publish(Bool(True))

        rospy.sleep(2.)

        rospy.loginfo('[%10s] Experiment Complete'%self.exp_string)
        
        return True    
    
    def save_data(self):
        save_path_1 = self.save_path + '_sensor1.npy'
        save_path_2 = self.save_path + '_sensor2.npy'
        save_path_tf = self.save_path + '_tf.npy'
        np.save(save_path_1, self.sensor_1_data)
        np.save(save_path_2, self.sensor_2_data)
        np.save(save_path_tf, self.target_force_data)
        rospy.loginfo('[%10s] Save complete at:  %s, %s, and %s'% ( self.node_string, save_path_1, save_path_2, save_path_tf))
        return True

    def shutdown(self, msg:Bool):
        if (msg.data):
            rospy.loginfo('[%10s] shutdown registered'%self.node_string )
            rospy.signal_shutdown('[%10s] shutdown registered'%self.node_string )

    def rosrun(self):
        while ( self.system_state == False ):
            self.system_check()
        
        self.experimental_procedure()

class UCDRobot_friction_estimation():

    '''
    This class implements the experimental procedure for the friction estimation experiments
    '''
    def __init__(self):
        
        self.node_string = 'UCD Robot'
        rospy.init_node('UCD Robot', anonymous=True)
        rospy.loginfo('[%10s] Starting %s'%( self.node_string, self.__class__.__name__))

        # Initiate hardware flags
        self.sensor_connect = [False, False]
        self.bias_state = False
        self.buffer_state = [False, False]
        self.sensor_state = [False, False]
        self.sensors_hub_state = False
        self.robot_state = False
        self.gripper_state = False
        self.camera_state = False
        self.system_state = False
        self.experiment_complete = False
        self.start_experiment = False

        # Initiate Variables
        self.i = [0, 0]
        self.measured_gfZ = np.array([0.0, 0.0])
        self.avg_measured_gfZ = 0
        self.prev_sensor_read = np.array([0, 0])
        self.init_z = -1
        self.z = 0

        # Read constants
        self.node_rate = rospy.get_param('NODE_RATE')
        self.GRIP_FORCE = rospy.get_param('GRIP_FORCE')
        self.FORCE_ACCURACY = rospy.get_param("FORCE_ACCURACY")
        self.EXP_NUM = rospy.get_param('EXP_NUM')
        self.GRIP_STEP = rospy.get_param('GRIP_STEP')

        # Read paths
        self.path = rospy.get_param('/save_path')
        self.object_type = rospy.get_param('/object_type')
        self.object_weight = rospy.get_param('/object_weight')
        self.object_distance = rospy.get_param('/object_distance')
        self.save_path = str(self.path) + '/' + str(self.EXP_NUM)  + '_' + str(self.GRIP_FORCE) + '_' + str(self.object_weight) + '_' + str(self.object_distance)

        # Initiate variables for data export
        self.sensor_1_data = {}
        self.sensor_2_data = {}
        self.target_force_data = {}

        # Initiate publishers
        self.pub_shutdown = rospy.Publisher('shutdown',Bool,queue_size=1)
        self.pub_system_state = rospy.Publisher('system_state',SystemState,queue_size=1)
        self.pub_gripper_move_state = rospy.Publisher('gripper_move_state',Bool,queue_size=1)
        self.pub_export_images = rospy.Publisher('export_images',String,queue_size=1)

        # Initiate controlling publishers
        self.pub_target_force = rospy.Publisher('target_force',TargetForce, queue_size=1)
        self.pub_target_pose = rospy.Publisher('target_pose',Pose, queue_size=1)

        # Initiate subscribers
        self.sub_sensor_1_state = rospy.Subscriber('/hub_0/sensor_0',SensorState,self.read_sensor_1)
        self.sub_sensor_2_state = rospy.Subscriber('/hub_0/sensor_1',SensorState,self.read_sensor_2)
        self.sub_robot_state = rospy.Subscriber('robot_state',Bool,self.read_robot_state)
        self.sub_gripper_state = rospy.Subscriber('gripper_state',Bool,self.read_gripper_state)
        self.sub_target_force = rospy.Subscriber('target_force',TargetForce,self.read_target_force)
        self.sub_camera_state = rospy.Subscriber('camera_state',Bool,self.read_camera_state)
        self.sub_program_running = rospy.Subscriber('/ur_hardware_interface/robot_program_running',Bool,self.read_robot_state)
        self.sub_joint_state = rospy.Subscriber('tf',TFMessage, self.read_joint_state)

        self.rate = rospy.Rate(self.node_rate)
        self.rosrun()

    def request_bias(self):
        rospy.wait_for_service("/hub_0/send_bias_request")
        bias_serv = rospy.ServiceProxy("/hub_0/send_bias_request",BiasRequest)
        resp = bias_serv.call()
        if resp.result:
            self.bias_state = True
            rospy.loginfo('[%10s] System Check: bias state %s'%( self.node_string, self.bias_state))
        else:
            raise RuntimeError('Bias sensor error')

    def start_slip_detection(self):
        rospy.wait_for_service("/hub_0/start_slip_detection")
        slip_serv = rospy.ServiceProxy("/hub_0/start_slip_detection",StartSlipDetection)
        resp = slip_serv.call()
        if resp.result:
            rospy.loginfo('[%10s] System Check: slip detection started'%( self.node_string))
        else:
            raise RuntimeError('Slip detection error')
        
    def system_check(self):

        system_state = SystemState()
        system_state.robot_state = self.robot_state
        system_state.gripper_state = self.gripper_state
        if (self.sensor_state[0] == self.sensor_state[1] ) and ( self.sensor_state[0] == True):
            system_state.sensor_state = self.sensor_state[0]
        system_state.camera_state = self.camera_state

        if ( self.sensor_state[0] ) and ( self.sensor_state[1] ) and ( self.robot_state ) and ( self.gripper_state ) and ( self.camera_state ) and ( not self.system_state ):
            rospy.loginfo('[%10s] System State: True'%(self.node_string))
            user_choice = 0 # 0 = ask, 1 = continue, -1 = stop
            # continue_test
            while user_choice == 0:
                input_str = input('[%10s] Start protocol? (1 = Yes, 0 = Stop)  \n'%(self.node_string))
                try:
                    choice = int(input_str)
                    if ( choice == 1):
                        rospy.loginfo('[%10s] User input: 1, continuing test'%self.node_string)
                        self.pub_gripper_move_state.publish(Bool(True))
                        user_choice = 1
                    elif ( choice == 0 ):
                        rospy.loginfo('[%10s] User input: 0, stopping test'%self.node_string)
                        self.pub_shutdown.publish(Bool(True))
                    else:
                        rospy.loginfo('[%10s] User input invalid'%self.node_string)
                except ValueError:
                    rospy.loginfo("Input is not a valid number. Please try again.")
            if user_choice == 1:            
                system_state.system_state = True
                self.system_state = True

        elif (not self.sensor_state[0] ) or ( not self.sensor_state[1])  or ( not self.robot_state ) or ( not self.gripper_state ) or ( not self.camera_state ):
            if ( self.system_state ) :
                if ( not self.sensor_state[0] ) or ( not self.sensor_state[1]):
                    rospy.loginfo('[%10s] Sensor State: %s'%(self.node_string,self.sensor_state))
                    self.system_state = False
                if ( not self.robot_state ):
                    rospy.loginfo('[%10s Robot State: %s'%(self.node_string,self.robot_state))
                    self.system_state = False
                if ( not self.gripper_state ):
                    rospy.loginfo('[%10s] Gripper State: %s'%(self.node_string,self.gripper_state))
                    self.system_state = False
                if ( not self.camera_state ):
                    rospy.loginfo('[%10s] Camera State: %s'%(self.node_string,self.camera_state))
                    self.system_state = False

            system_state.system_state = self.system_state
            self.pub_system_state.publish(system_state)

        else: 
            system_state.system_state = self.system_state

        self.pub_system_state.publish(system_state)

    def read_target_force(self, msg:TargetForce):
        if ( self.system_state ) and ( not self.experiment_complete):
            self.target_force = msg.target_force
            t = rospy.get_time()
            self.target_force_data[f'{t}'] = self.target_force

    def read_robot_state(self, msg:Bool):
        if ( msg.data ) and ( not self.robot_state ):
            rospy.loginfo('[%10s] Robot Connected'%self.node_string)
            self.robot_state = True
        elif ( not msg.data ) and ( self.robot_state ):
            rospy.loginfo('[%10s] Robot Disconnected'%self.node_string)       
            self.robot_state = False

    def read_gripper_state(self, msg:Bool):
        if ( msg.data ) and ( not self.gripper_state ):
            rospy.loginfo('[%10s] Gripper Connected'%self.node_string)
            self.gripper_state = True
        elif ( not msg.data ) and ( self.gripper_state ):
            rospy.loginfo('[%10s] Gripper Disconnected'%self.node_string)
            self.gripper_state = False

    def read_sensor_1(self, msg:SensorState):

        if ( not self.sensor_connect[0] ):
            rospy.loginfo('[%10s] Sensor 1 Connected'%self.node_string)
            self.request_bias()
            self.sensor_connect[0] = True

        if ( self.bias_state ) :
            self.i[0] += 1

        if ( self.i[0] > BUFFER_LENGTH ) and ( not self.buffer_state[0] ):
            rospy.loginfo('[%10s] Buffer 1 State: true'%self.node_string)
            self.buffer_state[0] = True
        
        if ( self.buffer_state[0] ) and ( self.bias_state ) and ( not self.sensor_state[0] ):
            rospy.loginfo('[%10s] Sensor 1 Ready'%self.node_string)
            self.sensor_state[0] = True

        if ( self.system_state ) and ( not self.experiment_complete):
            self.measured_gfZ[0] = msg.gfZ
            self.avg_measured_gfZ = np.mean(self.measured_gfZ) #(self.measured_gfZ[0] + self.measured_gfZ[1])/2.0
            t = rospy.get_time()
            self.sensor_1_data[f'{t}'] = msg
        
        self.prev_sensor_read[0] = rospy.get_time()

    def read_sensor_2(self, msg:SensorState):

        if ( not self.sensor_connect[1] ):
            rospy.loginfo('[%10s] Sensor 2 Connected'%self.node_string)
            self.request_bias()
            self.sensor_connect[1] = True

        if ( self.bias_state ) :
            self.i[1] += 1

        if ( self.i[1] > BUFFER_LENGTH ) and ( not self.buffer_state[1] ):
            rospy.loginfo('[%10s] Buffer 2 State: true'%self.node_string)
            self.buffer_state[1] = True
        
        if ( self.buffer_state[1] ) and ( self.bias_state ) and ( not self.sensor_state[1] ):
            rospy.loginfo('[%10s] Sensor 2 Ready'%self.node_string)
            self.sensor_state[1] = True

        if ( self.system_state ) and ( not self.experiment_complete):
            self.measured_gfZ[1] = msg.gfZ
            self.avg_measured_gfZ = np.mean(self.measured_gfZ) #(self.measured_gfZ[0] + self.measured_gfZ[1])/2.0
            t = rospy.get_time()
            self.sensor_2_data[f'{t}'] = msg
        
        self.prev_sensor_read[1] = rospy.get_time()

    def read_camera_state(self,msg:Bool):
        if ( msg.data ) and ( not self.camera_state ):
            rospy.loginfo('[%10s] Camera Connected'%self.node_string)
            self.camera_state = True

        elif ( not msg.data ) and ( self.camera_state ):
            rospy.loginfo('[%10s] Camera Disconnected'%(self.node_string))
            self.camera_state = False

    def read_joint_state(self, msg:TFMessage):
        if ( self.init_z == -1 ) :
            if (msg.transforms[0].transform.translation.z != 0 ) :
                self.init_z = msg.transforms[0].transform.translation.z
        
        if (msg.transforms[0].transform.translation.z != 0 ) :
            self.z = msg.transforms[0].transform.translation.z

    def friction_estimation(self):

        self.exp_string = 'Fric Est'
        self.target_force = self.GRIP_FORCE


        # ----------------- Apply grip Force------------------

        self.pub_gripper_move_state.publish(Bool(True))
        while ( np.abs( (self.target_force/2.0) - np.min(self.measured_gfZ) ) >= self.FORCE_ACCURACY ):
            target_force = TargetForce()
            target_force.target_force = float(self.target_force)
            self.pub_target_force.publish(target_force)

        rospy.sleep(2.0)

        rospy.loginfo("[%10s] Measured: %f target: %f"%(self.exp_string,np.min(self.measured_gfZ), self.target_force))

        rospy.sleep(2.0) 

        # ------------------- Raise robot-----------------

        rospy.loginfo('[%10s] Instructing robot to raise'%self.exp_string)
        up_pose = Pose(
            Vector3(0, 0, 0.08), Quaternion(0, 0, 0, 0)
        )
        self.pub_target_pose.publish(up_pose)
        while ( np.abs(self.z - self.init_z) < 0.078 ):
            print('up',np.abs(self.z - self.init_z) )
            rospy.sleep(1.)
        
        rospy.loginfo('[%10s] Robot raised'%(self.exp_string))

        self.start_slip_detection()
        
        # -------------------- Change grip force ------------------
        
        while ( self.target_force > 2 ) and ( np.min(self.measured_gfZ) > 1.0) : 

            self.target_force -= self.GRIP_STEP
            rospy.loginfo("[%10s] New Target Force: %f"%(self.exp_string,self.target_force))

            while ( np.abs( (self.target_force/2.0) - np.min(self.measured_gfZ) ) >= self.FORCE_ACCURACY ) and ( np.min(self.measured_gfZ) > 1.0):
                target_force = TargetForce()
                target_force.target_force = float(self.target_force)
                self.pub_target_force.publish(target_force)

            rospy.sleep(2.0)

            rospy.loginfo("[%10s] Measured: %f target: %f"%(self.exp_string,np.min(self.measured_gfZ), self.target_force))


        # ---------------------- Open gripper ----------------

        self.pub_target_force.publish(TargetForce(0.0))

        rospy.loginfo('[%10s] Instructing robot to open gripper'%self.exp_string)

        #---------------------- Lower robot ----------------

        user_choice = 0 # 0 = ask, 1 = continue, -1 = stop
        # continue_test
        while user_choice == 0:
            input_str = input('[%10s] Safe to lower robot? (1 = Yes, 0 = Stop)  \n'%(self.exp_string))
            try:
                choice = int(input_str)
                if ( choice == 1):
                    rospy.loginfo('[%10s] User input: 1, continuing test'%self.exp_string)
                    user_choice = 1
                elif ( choice == 0 ):
                    rospy.loginfo('[%10s] User input: 0, stopping test'%self.exp_string)
                    # user_choice = -1
                else:
                    rospy.loginfo('[%10s] User input invalid'%self.exp_string)
            except ValueError:
                rospy.loginfo("Input is not a valid number. Please try again.")

        rospy.loginfo('[%10s] instructing robot to lower'%(self.exp_string))

        # Move robot
        down_pose = Pose(
            Vector3(0, 0, -0.08), Quaternion(0, 0, 0, 0)
        )
        self.pub_target_pose.publish(down_pose)
        rospy.sleep(1.0)

        rospy.loginfo('[%10s] robot lowered'%(self.exp_string))

        # --------------------------- End --------------------

        rospy.loginfo('[%10s] test complete'%(self.exp_string))

        user_choice = 0 # 0 = ask, 1 = continue, -1 = stop
        # continue_test
        while user_choice == 0:
            input_str = input('[%10s] Save data? (1 = Yes, 0 = Stop)  \n'%(self.exp_string))
            try:
                choice = int(input_str)
                if ( choice == 1):
                    rospy.loginfo('[%10s] User input: 1, saving data'%self.exp_string)
                    user_choice = 1
                elif ( choice == 0 ):
                    rospy.loginfo('[%10s] User input: 0, stopping test'%self.exp_string)
                    # user_choice = -1
                else:
                    rospy.loginfo('[%10s] User input invalid'%self.exp_string)
            except ValueError:
                rospy.loginfo("Input is not a valid number. Please try again.")

        export_string = self.exp_string
        self.pub_export_images.publish(export_string)

        self.system_state = False

        save_sensor_data = False
        while not save_sensor_data:
            save_sensor_data = self.save_data()

        self.pub_shutdown.publish(Bool(True))

        rospy.sleep(3.)

        rospy.loginfo('[%10s] Experiment Complete'%self.exp_string)
        
        return True  
    
    def save_data(self):
        save_path_1 = self.save_path + '_sensor1.npy'
        save_path_2 = self.save_path + '_sensor2.npy'
        save_path_tf = self.save_path + '_tf.npy'
        np.save(save_path_1, self.sensor_1_data)
        np.save(save_path_2, self.sensor_2_data)
        np.save(save_path_tf, self.target_force_data)
        rospy.loginfo('[%10s] Save complete at:  %s, %s, and %s'% ( self.node_string, save_path_1, save_path_2, save_path_tf))
        return True
    
    def rosrun(self):
        while ( self.system_state == False ):
            self.system_check()
        
        test_complete = False
        while not test_complete:
            test_complete = self.friction_estimation()
            # rospy.spin()
        self.pub_shutdown.publish(Bool(True))
        rospy.sleep(1.0)

if __name__ == '__main__':
    try:
        hand = UCDRobot_const()
        # hand = UCDRobot_react()
        # hand = UCDRobot_friction_estimation()

    except rospy.ROSInterruptException:
        pass