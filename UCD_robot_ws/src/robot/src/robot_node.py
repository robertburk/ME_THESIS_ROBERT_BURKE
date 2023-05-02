#!/usr/bin/env python3

'''
Author: Robert Burke <robertljburke@gmail.com>

Github: @robertburk

This is the main class you can use for controlling the robot

'''

import sys
import rospy
import numpy as np
import pandas as pd

from std_msgs.msg import Bool
import actionlib
from tf2_msgs.msg import TFMessage
from control_msgs.msg import FollowJointTrajectoryAction, FollowJointTrajectoryGoal
from trajectory_msgs.msg import JointTrajectoryPoint
from std_srvs.srv import Trigger
from controller_manager_msgs.srv import SwitchControllerRequest, SwitchController
from controller_manager_msgs.srv import LoadControllerRequest, LoadController
from controller_manager_msgs.srv import ListControllers, ListControllersRequest
from ucd_robot.msg import SystemState
from geometry_msgs.msg import Pose, Vector3, Quaternion, Accel
from cartesian_control_msgs.msg import (
    FollowCartesianTrajectoryAction,
    FollowCartesianTrajectoryGoal,
    CartesianTrajectoryPoint,
)

# If your robot description is created with a tf_prefix, those would have to be adapted
JOINT_NAMES = [
    "shoulder_pan_joint",
    "shoulder_lift_joint",
    "elbow_joint",
    "wrist_1_joint",
    "wrist_2_joint",
    "wrist_3_joint",
]
JOINT_TRAJECTORY_CONTROLLERS = [
    "scaled_pos_joint_traj_controller",
    "scaled_vel_joint_traj_controller",
    "pos_joint_traj_controller",
    "vel_joint_traj_controller",
    "forward_joint_traj_controller",
]
CARTESIAN_TRAJECTORY_CONTROLLERS = [
    "pose_based_cartesian_traj_controller",
    "joint_based_cartesian_traj_controller",
    "forward_cartesian_traj_controller",
]
CONFLICTING_CONTROLLERS = ["joint_group_vel_controller", "twist_controller"]

UR5E_RATE = 1000

class UR5e_controller():
    def __init__(self):
        self.node_string = 'Robot'
        rospy.init_node('Robot', anonymous=True)
        rospy.loginfo('[%10s] Starting %s'%( self.node_string, self.__class__.__name__))

        # Initialise state variables
        self.robot_state = False
        self.system_state = False

        self.cartesian_trajectory_controller = CARTESIAN_TRAJECTORY_CONTROLLERS[0]
        rospy.sleep(5.0)
        # Initialise Controllers
        timeout = rospy.Duration(5)
        self.switch_srv = rospy.ServiceProxy(
            "controller_manager/switch_controller", SwitchController
        )
        self.load_srv = rospy.ServiceProxy("controller_manager/load_controller", LoadController)
        self.list_srv = rospy.ServiceProxy("controller_manager/list_controllers", ListControllers)
        try:
            self.switch_srv.wait_for_service(timeout.to_sec())
        except rospy.exceptions.ROSException as err:
            rospy.logerr("Could not reach controller switch service. Msg: {}".format(err))
            sys.exit(-1)

        self.load_prog = rospy.ServiceProxy("ur_hardware_interface/dashboard/play",Trigger)
        try:
            self.load_prog.wait_for_service(timeout.to_sec())
        except rospy.exceptions.ROSException as err:
            rospy.logerr("Could not reach load prog service. Msg: {}".format(err))
            sys.exit(-1)
        self.load_prog.call()

        # Initialise position values
        self.x_t = 0
        self.y_t = 0
        self.z_t = 0
        self.x_r = 0
        self.y_r = 0
        self.z_r = 0
        self.w_r = 0

        # Read Params
        self.node_rate = rospy.get_param('NODE_RATE')

        # Initiate subscribers
        self.sub_shutdown = rospy.Subscriber('shutdown',Bool,self.shutdown)
        self.sub_joint_state = rospy.Subscriber('tf',TFMessage, self.joint_states)
        self.sub_robot_state = rospy.Subscriber('robot_state',Bool,self.read_robot_state_1)
        self.sub_target_pose = rospy.Subscriber('target_pose',Pose, self.send_cartesian_trajectory)
        self.sub_system_state = rospy.Subscriber('system_state',SystemState,self.read_system_state)
        self.sub_program_running = rospy.Subscriber('/ur_hardware_interface/robot_program_running',Bool,self.read_robot_state)

        self.rate = rospy.Rate(self.node_rate)
        self.rosrun()

    def read_system_state(self,msg:SystemState):
        if ( msg.system_state ) and ( not self.system_state ):
            rospy.loginfo('[%10s] System State: True',self.node_string)
            self.system_state = True

        elif ( not msg.system_state ) and ( self.system_state ):
            rospy.loginfo('[%10s] System State: False',self.node_string)
            self.system_state = False
            rospy.sleep(1.0)
            self.load_prog.call()

        elif ( not msg.system_state ) and ( not self.robot_state ):
            self.load_prog.call()

    def read_robot_state_1(self,msg:Bool):
        if ( msg.data ) and ( not self.robot_state ):
            rospy.loginfo('[%10s] Robot State: %s'%(self.node_string,msg.data))
            self.robot_state = msg.data
        
        elif ( not msg.data ) and ( self.robot_state ):
            rospy.loginfo('[%10s] Robot State: %s'%(self.node_string,msg.data))
            # self.robot_state = msg.data
            rospy.sleep(1.0)
            self.load_prog.call()
            # self.robot_state = True

    def read_robot_state(self,msg:Bool):
        if ( msg.data ) and ( not self.robot_state ):
            rospy.loginfo('[%10s] Robot State: %s'%(self.node_string,msg.data))
            self.robot_state = msg.data
        
        elif ( not msg.data ) and ( self.robot_state ):
            rospy.loginfo('[%10s] Robot State: %s'%(self.node_string,msg.data))
            # self.robot_state = msg.data
            rospy.sleep(1.0)
            self.load_prog.call()
            # self.robot_state = True

    def initialise_controllers(self):
        # Initialise Controllers
        timeout = rospy.Duration(5)
        self.switch_srv = rospy.ServiceProxy(
            "controller_manager/switch_controller", SwitchController
        )
        self.load_srv = rospy.ServiceProxy("controller_manager/load_controller", LoadController)
        self.list_srv = rospy.ServiceProxy("controller_manager/list_controllers", ListControllers)

        try:
            self.switch_srv.wait_for_service(timeout.to_sec())
        except rospy.exceptions.ROSException as err:
            rospy.logerr("Could not reach controller switch service. Msg: {}".format(err))
            sys.exit(-1)
        self.cartesian_trajectory_controller = CARTESIAN_TRAJECTORY_CONTROLLERS[0]
        self.switch_controller(self.cartesian_trajectory_controller)

    def joint_states(self, msg:TFMessage):
        if ( msg.transforms[0].header.frame_id == "base" ) :
            if ( msg.transforms[0].transform.translation.x != 0 ):
                self.x_t = msg.transforms[0].transform.translation.x
            if ( msg.transforms[0].transform.translation.y != 0 ):
                self.y_t = msg.transforms[0].transform.translation.y
            if ( msg.transforms[0].transform.translation.z != 0 ):
                self.z_t = msg.transforms[0].transform.translation.z
            if ( msg.transforms[0].transform.rotation.x != 0 ):
                self.x_r = msg.transforms[0].transform.rotation.x
            if ( msg.transforms[0].transform.rotation.y != 0 ):
                self.y_r = msg.transforms[0].transform.rotation.y
            if ( msg.transforms[0].transform.rotation.z != 0 ):
                self.z_r = msg.transforms[0].transform.rotation.z
            if ( msg.transforms[0].transform.rotation.w != 0 ):
                self.w_r = msg.transforms[0].transform.rotation.w

    def send_cartesian_trajectory(self, msg:Pose):
        """Creates a Cartesian trajectory and sends it using the selected action server"""

        if ( self.system_state ):

            """Creates a Cartesian trajectory and sends it using the selected action server"""
            self.switch_controller(self.cartesian_trajectory_controller)

            # make sure the correct controller is loaded and activated
            goal = FollowCartesianTrajectoryGoal()
            trajectory_client = actionlib.SimpleActionClient(
                "{}/follow_cartesian_trajectory".format(self.cartesian_trajectory_controller),
                FollowCartesianTrajectoryAction,
            )
            # Wait for action server to be ready
            timeout = rospy.Duration(5)
            if not trajectory_client.wait_for_server(timeout):
                rospy.logerr("Could not reach controller action server.")
                sys.exit(-1)

            new_pose = Pose(
                Vector3(self.x_t,self.y_t,self.z_t + msg.position.z), Quaternion(self.x_r, self.y_r,self.z_r, self.w_r)
            )
            
            duration_list = 4.0
            point = CartesianTrajectoryPoint()
            point.pose = new_pose
            point.time_from_start = rospy.Duration(duration_list)
            goal.trajectory.points.append(point)

            rospy.loginfo(
                "Executing trajectory using the {}".format(self.cartesian_trajectory_controller)
            )
            # rospy.sleep(0.5)
            trajectory_client.send_goal(goal)
            trajectory_client.wait_for_result()

            result = trajectory_client.get_result()

            rospy.loginfo("Trajectory execution finished in state {}".format(result.error_code))

    def switch_controller(self, target_controller):
        """Activates the desired controller and stops all others from the predefined list above"""
        other_controllers = (
            JOINT_TRAJECTORY_CONTROLLERS
            + CARTESIAN_TRAJECTORY_CONTROLLERS
            + CONFLICTING_CONTROLLERS
        )

        other_controllers.remove(target_controller)

        srv = ListControllersRequest()
        response = self.list_srv(srv)
        for controller in response.controller:
            if controller.name == target_controller and controller.state == "running":
                return

        srv = LoadControllerRequest()
        srv.name = target_controller
        self.load_srv(srv)

        srv = SwitchControllerRequest()
        srv.stop_controllers = other_controllers
        srv.start_controllers = [target_controller]
        srv.strictness = SwitchControllerRequest.BEST_EFFORT
        self.switch_srv(srv)

    def shutdown(self, msg:Bool):
        if ( msg.data ) :
            rospy.loginfo('[%10s] shutdown registered'%self.node_string)
            rospy.signal_shutdown('[%10s] shutdown registered'%self.node_string)

    def rosrun(self):
        rospy.spin()

if __name__ == '__main__':
    try:
        hand = UR5e_controller()

    except rospy.ROSInterruptException:
        pass