#!/usr/bin/env python3

'''
Author: Robert Burke <robertljburke@gmail.com>

Github: @robertburk

'''

import rospy
import numpy as np

from std_msgs.msg import Bool
from ucd_robot.msg import SystemState
from gripper.msg import TargetForce
from papillarray_ros_v2.msg import SensorState
from gripper.msg import TargetDelta
from geometry_msgs.msg import Pose, Vector3, Quaternion

x_dis                           = 7
DX_INIT                         = [7, 0, -7, 7, 0, -7, 7, 0, -7]
DY_INIT                         = [7, 7, 7, 0, 0, 0, -7, -7, -7]
DZ_INIT                         = [0.5, 0.2, 0.5, 0.2, 0.0, 0.2, 0.5, 0.2, 0.5]


MU                              = 0.3
TRANS_SAFETY_MARGIN             = 1.15
ROT_SAFETY_MARGIN               = 1.2
PROTOCOL_THRESHOLD              = 1.1
TORQUE_PROTOCOL_THRESHOLD       = 1.05
SENSOR_CONTACT_THRESHOLD        = 1
EXPLORE_TF                      = 4
MIN_FORCE                       = 4


class GripForceProtocol_react():

    '''

    This class implements a reactive grip force protocol that maintains maximum traction 
    above 1.15X the experienced tangential force load and 1.2X the load torque

    '''

    def __init__(self):

        self.node_string = 'GF Prot.'
        rospy.init_node('Grip Force Protocol', anonymous=True)
        rospy.loginfo('[%10s] Starting %s'%( self.node_string, self.__class__.__name__))

        # Initialise state variables
        self.system_state = False
        self.sensor_1_state = False
        self.sensor_2_state = False
        self.sensor_1_contact = False
        self.sensor_2_contact = False
        self.explore_mode = True
        self.robot_pose = 'down'
        self.reactive_mode = False
        self.open = False

        # Initialise Variables
        self.measured_fZ = np.array([0.0,0.0])
        self.avg_measured_fZ = 0
        self.target_force = np.array([0.0,0.0])
        self.prev_target_force = 0
        self.error = 0
        self.start_time = 0

        # Read Params
        self.MAX_FORCE = rospy.get_param('MAX_FORCE')
        self.node_rate = rospy.get_param('NODE_RATE')

        # Initiate Publishers
        self.pub_target_force = rospy.Publisher('target_force',TargetForce,queue_size=1)
        # Initiate subscribers
        self.sub_shutdown = rospy.Subscriber('shutdown',Bool,self.shutdown)
        self.sub_system_state = rospy.Subscriber('system_state',SystemState,self.read_system_state)
        self.sub_sensor_1_state = rospy.Subscriber('/hub_0/sensor_0',SensorState,self.read_sensor_1)
        self.sub_sensor_2_state = rospy.Subscriber('/hub_0/sensor_1',SensorState,self.read_sensor_2)
        self.sub_target_pose = rospy.Subscriber('target_pose',Pose, self.read_robot_pose)
        
        self.rate = rospy.Rate(self.node_rate)
        self.rosrun()

    def read_system_state(self, msg:SystemState):
        if ( msg.system_state ) and ( not self.system_state ):
            rospy.loginfo('[%10s] System State: True'%self.node_string)
            self.start_time = rospy.get_time()
            self.system_state = True

        elif ( not msg.system_state ) and ( self.system_state ):
            rospy.loginfo('[%10s] System State: False'%self.node_string)
            self.system_state = False

    def read_sensor_1(self, msg:SensorState):
        if ( not self.sensor_1_state ):
            self.sensor_1_state         = True

        if ( not self.explore_mode ) and ( self.measured_fZ[0] < 0.25 ):
                self.open = True

        if ( self.system_state ) and ( not self.open ):

            self.measured_fZ[0]         = msg.gfZ

            if ( msg.is_contact and self.explore_mode ):

                self.sensor_1_contact   = True

                if self.sensor_2_contact:

                    self.explore_mode       = False           

            if not self.explore_mode:


                ft_mag                  = 0
                gtZ                     = 0
                max_trans_traction      = 0
                max_rot_traction        = 0
                fn_calc                 = np.array([0.0,0.0])

                for i in range(9):
                    pillar      = msg.pillars[i]
                    tz          = 0
                    fx          = pillar.fX
                    fy          = pillar.fY
                    fz          = pillar.fZ

                    if ( pillar.in_contact ):

                        rx                  = DX_INIT[i] + pillar.dX
                        ry                  = DY_INIT[i] + pillar.dY

                        ft_mag_p            = np.sqrt( np.square( fx ) + np.square( fy ) )
                        ft_ang_p            = np.arctan2( fx , fy )

                        r_mag_p             = np.sqrt( np.square( rx ) + np.square( ry ) )
                        r_ang_p             = np.arctan2( rx , ry)

                        tz                  = ( r_mag_p * ft_mag_p * np.sin(r_ang_p - ft_ang_p))
                        fn_calc[1]          += np.abs(tz / ( r_mag_p * MU * np.sin(r_ang_p - ft_ang_p)) )

                        gtZ                 += tz
                        max_rot_traction    += (r_mag_p * MU * fz * (np.sin(r_ang_p - ft_ang_p)) )
                        
                        fn_calc[0]          += ft_mag_p/MU
                        
                        ft_mag              += ft_mag_p 
                        

                        max_trans_traction  += ( fz ) * ( MU )

                gtZ                 = TORQUE_PROTOCOL_THRESHOLD*np.abs(msg.gtZ) 
                max_rot_traction    = np.abs(max_rot_traction)
                ft_mag              = PROTOCOL_THRESHOLD*ft_mag
                fn_calc[0]          = TRANS_SAFETY_MARGIN*fn_calc[0]  
                fn_calc[1]          = ROT_SAFETY_MARGIN*fn_calc[1]

                ft_flag = ft_mag > max_trans_traction
                torque_flag = gtZ > max_rot_traction
                
                if  ( ft_flag ) and ( torque_flag):
                    self.target_force[0]                = np.max(fn_calc)
                
                elif ( not ft_flag ) and ( torque_flag ):
                    self.target_force[0]                = fn_calc[1]

                else:
                    self.target_force[0]                = fn_calc[0]
            
            else:
                self.target_force[0]    =   2*EXPLORE_TF

    def read_sensor_2(self, msg:SensorState):
        if ( not self.sensor_2_state ):
            self.sensor_2_state = True

        if ( not self.explore_mode ) and ( self.measured_fZ[1] < 0.25 ):
            self.open = True

        if ( self.system_state ) and ( not self.open ):

            self.measured_fZ[1]         = msg.gfZ

            if ( msg.is_contact and self.explore_mode ):

                self.sensor_2_contact   = True

                if self.sensor_1_contact:

                    self.explore_mode       = False                

            if not self.explore_mode:
                ft_mag                  = 0
                gtZ                     = 0
                max_trans_traction      = 0
                max_rot_traction        = 0
                fn_calc                 = np.array([0.0,0.0])

                for i in range(9):
                    pillar      = msg.pillars[i]
                    fx          = pillar.fX
                    fy          = pillar.fY
                    fz          = pillar.fZ
                    tz = 0

                    if ( pillar.in_contact ):

                        rx                  = DX_INIT[i] + pillar.dX
                        ry                  = DY_INIT[i] + pillar.dY

                        ft_mag_p            = np.sqrt( np.square( fx ) + np.square( fy ) )
                        ft_ang_p            = np.arctan2( fx , fy )

                        r_mag_p             = np.sqrt( np.square( rx ) + np.square( ry ) )
                        r_ang_p             = np.arctan2( rx , ry)

                        tz                  = ( r_mag_p * ft_mag_p * np.sin(r_ang_p - ft_ang_p))
                        fn_calc[1]          += np.abs(tz / ( r_mag_p * MU * np.sin(r_ang_p - ft_ang_p)) )

                        max_rot_traction    += (r_mag_p * MU * fz * (np.sin(r_ang_p - ft_ang_p)) )
                        gtZ                 += tz
                                                
                        fn_calc[0]          += ft_mag_p/MU
                        
                        ft_mag              += ft_mag_p 

                        max_trans_traction  += ( fz ) * ( MU )

                gtZ                 = TORQUE_PROTOCOL_THRESHOLD*np.abs(msg.gtZ) 
                max_rot_traction    = np.abs(max_rot_traction)
                ft_mag              = PROTOCOL_THRESHOLD*ft_mag
                fn_calc[0]          = TRANS_SAFETY_MARGIN*fn_calc[0]  
                fn_calc[1]          = ROT_SAFETY_MARGIN*fn_calc[1]  
                
                ft_flag = ft_mag > max_trans_traction
                torque_flag = gtZ > max_rot_traction
                
                if  ( ft_flag ) and ( torque_flag):
                    self.target_force[1]                = np.max(fn_calc)

                elif ( not ft_flag ) and ( torque_flag ):
                    self.target_force[1]                = fn_calc[1]

                else:
                    self.target_force[1]                = fn_calc[0]
            
            else:
                self.target_force[1]    =   2*EXPLORE_TF

    def read_robot_pose(self, msg:Pose):
        if ( self.robot_pose == 'down'):
            self.robot_pose = 'up'
        else:
            self.robot_pose = 'down' 

    def publish_target_force(self):
        if ( self.system_state ):
            if ( self.robot_pose == 'down' ) and ( not self.open ):
                target_force = TargetForce(2*MIN_FORCE)
            elif ( rospy.get_time() - self.start_time < 15 ) and ( not self.open ):
                target_force = TargetForce()        

                target = 2*np.max(self.target_force)   

                if ( target >= 2*MIN_FORCE ):
                    target_force.target_force       = target

                elif ( self.explore_mode ):
                    target_force.target_force       = 2*EXPLORE_TF

                else: 
                    target_force.target_force       = 2*MIN_FORCE            
            
            else:
                target_force = TargetForce()
                target_force.target_force = 0.0

            self.pub_target_force.publish(target_force)

            if ( self.prev_target_force != target_force.target_force ):
                rospy.loginfo('[%10s] target force: %f'%(self.node_string, target_force.target_force))
                self.prev_target_force          = target_force.target_force                    

    def shutdown(self, msg:Bool):
        if (msg.data):
            rospy.loginfo('[%10s] shutdown registered'%self.node_string )
            rospy.signal_shutdown('[%10s] shutdown registered'%self.node_string )

    def rosrun(self):
        while not rospy.is_shutdown():
            self.publish_target_force()
            # rospy.spin()

class GripForceProtocol_const():

    '''
    This class implements a fixed grip force protocol that maintains the grip force at the target
    '''

    def __init__(self):

        self.node_string = 'GF Prot.'
        rospy.init_node('Grip Force Protocol', anonymous=True)
        rospy.loginfo('[%10s] Starting %s'%( self.node_string, self.__class__.__name__))

        # Initialise state variables
        self.system_state = False
        self.sensor_1_state = False
        self.sensor_2_state = False
        self.sensor_1_contact = False
        self.sensor_2_contact = False
        self.explore_mode = True
        self.open = False
        self.start_time = 0

        # Initialise Variables
        self.measured_fZ = np.array([0.0,0.0])
        self.avg_measured_fZ = 0
        self.target_force = np.array([0.0,0.0])
        self.prev_target_force = 0
        self.error = 0

        # Read Params
        self.MAX_FORCE = rospy.get_param('MAX_FORCE')
        self.node_rate = rospy.get_param('NODE_RATE')
        self.GRIP_FORCE = rospy.get_param('GRIP_FORCE')

        # Initiate Publishers
        self.pub_target_force = rospy.Publisher('target_force',TargetForce,queue_size=1)
        # Initiate subscribers
        self.sub_shutdown = rospy.Subscriber('shutdown',Bool,self.shutdown)
        self.sub_system_state = rospy.Subscriber('system_state',SystemState,self.read_system_state)
        self.sub_sensor_1_state = rospy.Subscriber('/hub_0/sensor_0',SensorState,self.read_sensor_1)
        self.sub_sensor_2_state = rospy.Subscriber('/hub_0/sensor_1',SensorState,self.read_sensor_2)

        self.rate = rospy.Rate(self.node_rate)
        self.rosrun()

    def read_system_state(self, msg:SystemState):
        if ( msg.system_state ) and ( not self.system_state ):
            rospy.loginfo('[%10s] System State: True'%self.node_string)
            self.start_time = rospy.get_time()
            self.system_state = True

        elif ( not msg.system_state ) and ( self.system_state ):
            rospy.loginfo('[%10s] System State: False'%self.node_string)
            self.system_state = False

    def read_sensor_1(self, msg:SensorState):
        if ( not self.sensor_1_state ):
            self.sensor_1_state = True

        if ( self.system_state):

            self.measured_fZ[0] = msg.gfZ

            if ( msg.gfZ > SENSOR_CONTACT_THRESHOLD and self.explore_mode ):
                self.sensor_1_contact = True
                if self.sensor_2_contact:
                    self.explore_mode = False

            if ( not self.explore_mode ) and ( self.measured_fZ[0] < 0.25 ):
                self.open = True

    def read_sensor_2(self, msg:SensorState):
        if ( not self.sensor_2_state ):
            self.sensor_2_state = True

        if ( self.system_state ):
            self.measured_fZ[1] = msg.gfZ

            if ( msg.gfZ > SENSOR_CONTACT_THRESHOLD and self.explore_mode ):
                self.sensor_2_contact = True
                if self.sensor_1_contact:
                    self.explore_mode = False

            if ( not self.explore_mode ) and ( self.measured_fZ[1] < 0.25 ):
                self.open = True

    def publish_target_force(self):
        if ( self.system_state ):
            target_force = TargetForce()

            if ( rospy.get_time() - self.start_time < 20 ) and ( not self.open ):
                target_force.target_force = self.GRIP_FORCE

            else:
                target_force.target_force = 0.0

            self.pub_target_force.publish(target_force)
            if ( self.prev_target_force != target_force.target_force ):
                rospy.loginfo('[%10s] target force: %f'%(self.node_string, target_force.target_force))
                self.prev_target_force = target_force.target_force
                
    def shutdown(self, msg:Bool):
        if (msg.data):
            rospy.loginfo('[%10s] shutdown registered'%self.node_string )
            rospy.signal_shutdown('[%10s] shutdown registered'%self.node_string )

    def rosrun(self):
        while not rospy.is_shutdown():
            self.publish_target_force()
            # rospy.spin()

if __name__ == '__main__':
    try:
        # hand = GripForceProtocol_const()
        hand = GripForceProtocol_react()

    except rospy.ROSInterruptException:
        pass