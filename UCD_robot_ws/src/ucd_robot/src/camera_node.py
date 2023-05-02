#!/usr/bin/env python3
# -*- coding: utf-8 -*-
'''
Author: Robert Burke <robertljburke@gmail.com>

Github: @robertburk

This is the camera node, which continuously records images. 

'''
import numpy as np
import warnings
import datetime
import os.path
import rospy
import cv2
from cv2 import aruco
import pyrealsense2 as rs

from papillarray_ros_v2.msg import SensorState
from geometry_msgs.msg import Pose
from std_msgs.msg import Bool, String
from ucd_robot.msg import SystemState
from gripper.msg import TargetForce

warnings.filterwarnings("ignore")


'''
Modify this if you want to make the image clearer.
'''
width = 960
height  = 540
FONT = 'HERSHEY_DUPLEX'
FPS = 50.0
FRAME_RATE = 1.0/FPS

PLOT_RATE = 1.0/20.0

port_num = 6 # 6 for USB3, 4 for USB2

class RealSenseCamera():
    def __init__(self):
        self.node_string = 'RealSense'
        rospy.init_node('Camera', anonymous=True)
        rospy.loginfo('[%10s] Starting %s'%( self.node_string, self.__class__.__name__))

        # Initiate Flags 
        self.camera_ready = False
        self.system_state = False
        self.robot_state = 'down'
        self.robot_raise_time = 0
        self.robot_lower_time = 0
        self.last_frame_time = 0
        self.last_publish_time = 0

        # Initiate camera Variables
        pipeline = rs.pipeline()
        config = rs.config()
        config.enable_stream(rs.stream.color, width, height, rs.format.bgr8, 30)
        pipeline.start(config)
        align_to = rs.stream.color
        align = rs.align(align_to)
        frames = pipeline.wait_for_frames(timeout_ms=10000)
        aligned_frames = align.process(frames)
        color_frame = aligned_frames.get_color_frame()
        intr = color_frame.profile.as_video_stream_profile().intrinsics
        intr_matrix = np.array([
            [intr.fx, 0, intr.ppx], [0, intr.fy, intr.ppy], [0, 0, 1]
        ])
        pipeline.stop()
        params = {}
        params['matrix'] = intr_matrix
        params['coeffs'] = np.array(intr.coeffs)
        
        # Initiate Aruco Variables
        self.camera = cv2.VideoCapture(port_num)
        self.camera.set(cv2.CAP_PROP_BUFFERSIZE, 1)
        self.camera.set(3, width)  # width=1920
        self.camera.set(4, height)  # height=1080
        

        self.fps = self.camera.get(cv2.CAP_PROP_FPS)
        self.frame_rate = 1.0/self.fps

        self.arucoDict = aruco.getPredefinedDictionary(aruco.DICT_4X4_50)
        self.arucoParams = aruco.DetectorParameters()
        self.detector = aruco.ArucoDetector(self.arucoDict, self.arucoParams)

        # Read Constants
        self.node_rate = rospy.get_param('NODE_RATE')

        # Read params
        self.path = rospy.get_param('/save_path')
        self.object_type = rospy.get_param('/object_type')
        self.object_weight = rospy.get_param('/object_weight')
        self.object_distance = rospy.get_param('/object_distance')
        self.GRIP_FORCE = rospy.get_param('GRIP_FORCE')
        self.EXP_NUM = rospy.get_param('EXP_NUM')

        # Initiate path variables
        self.images = {}
        self.data = {}
        self.distances = {}
        self.data['params'] = params
        self.csv_path = self.path + '/exp_data.csv'

        self.file_name = str(self.path) + '/' + str(self.EXP_NUM) + '_' + str(self.GRIP_FORCE) + '_' + str(self.object_weight) + '_' + str(self.object_distance)
        self.csv_columns = ['filename','date','object','weight','distance','raise_time','lower_time']

        # Initiate variables
        self.measured_fZ = np.array([0.0,0.0])
        self.target_force = 0.0
        self.contactile_tf = np.array([0.0,0.0])

        # Initialise Publishers
        self.pub_camera_state = rospy.Publisher('camera_state',Bool,queue_size=1)

        # Initialise Subscribers
        self.sub_shutdown = rospy.Subscriber('shutdown',Bool,self.shutdown)
        self.sub_system_state = rospy.Subscriber('system_state',SystemState,self.read_system_state)
        self.sub_export_images = rospy.Subscriber('export_images',String,self.save_data)
        self.sub_target_pose = rospy.Subscriber('target_pose',Pose, self.read_robot_state)
        self.sub_target_force = rospy.Subscriber('target_force',TargetForce,self.read_target_force)
        self.sub_sensor_1_state = rospy.Subscriber('/hub_0/sensor_0',SensorState,self.read_sensor_1)
        self.sub_sensor_2_state = rospy.Subscriber('/hub_0/sensor_1',SensorState,self.read_sensor_2)

        self.init_camera()

        self.rate = rospy.Rate(self.node_rate)
        self.rosrun()

    def init_camera(self):
        '''
        This function is crucial because the camera needs to be initialized 
        through several steps the first time it's used after starting up. 
        Otherwise, the initial frames captured may not be clear.
        '''
        for i in range(100):
            self.camera.read()

        self.camera_ready = True
        self.pub_camera_state.publish(self.camera_ready)
    
    def read_system_state(self,msg:SystemState):
        if ( msg.system_state ) and ( not self.system_state ):
            rospy.loginfo('[%10s] System State: True'%self.node_string)
            self.system_state = True

        elif ( not msg.system_state ) and ( self.system_state ):
            rospy.loginfo('[%10s] System State: False'%(self.node_string))
            self.system_state = False
    
    def read_target_force(self, msg:TargetForce):
        self.target_force = msg.target_force/2.0
  
    def read_sensor_1(self, msg:SensorState): 
        self.measured_fZ[0] = round(msg.gfZ ,2)
        self.contactile_tf[0] = round(msg.target_grip_force,2)
        self.update()
    
    def read_sensor_2(self, msg:SensorState): 
        self.measured_fZ[1] = round(msg.gfZ,2)
        self.contactile_tf[1] = round(msg.target_grip_force,2)
        
    def read_robot_state(self, msg:Pose):
        if ( self.robot_state == 'down'):
            self.robot_raise_time = rospy.get_time()
            self.robot_state = 'up'
        else:
            self.robot_lower_time = rospy.get_time()
            self.robot_state = 'down' 
            # cv2.destroyAllWindows()
            self.system_state = False

    def save_data(self, msg:String):

        self.system_state = False

        self.data['images'] = self.images

        # Write the text file
        date_string = str(datetime.date.today().year) + '-' + str(datetime.date.today().month) + '-' + str(datetime.date.today().day)
        csv_row = str(self.file_name) + ',' +  str(date_string) + ',' + str(self.object_type) + ',' + str(self.object_weight) + ',' + str(self.object_distance) + ',' + str(self.robot_raise_time) + ',' + str(self.robot_lower_time)

        if ( os.path.isfile(self.csv_path)):
            file = open(self.csv_path,'a')
            file.write("\n")
        else:
            file = open(self.csv_path,'a')
        file.write(csv_row)
        file.close()  

        # Write the images file
        camera_data_filename = f'{self.file_name}_camera_data.npy'
        np.save(camera_data_filename, self.data)
        rospy.loginfo('[%10s] Save succesful to: %s'% ( self.node_string, camera_data_filename ) )

        distance_data_filename = f'{self.file_name}_distance_data.npy'
        np.save(distance_data_filename, self.distances)
        rospy.loginfo('[%10s] Save succesful to: %s'% ( self.node_string, distance_data_filename ) )

        return True
    
    def update(self):
        if ( rospy.get_time() - self.last_frame_time > self.frame_rate ):
            _, frame = self.camera.read()
            if ( self.system_state ) :
                corners, ids, _ = self.detector.detectMarkers(frame)
                if len(list(corners)) == 2:

                    # ------------------ Calculate the distance between the gripper and the object ------------------

                    gripper_corners = corners[0].reshape((4,2))
                    object_corners = corners[1].reshape((4,2))

                    coordinate_conversion0 = 20 / np.sqrt((gripper_corners[0][0] - gripper_corners[1][0])**2 + (gripper_corners[0][1] - gripper_corners[1][1])**2)
                    coordinate_conversion1 = 20 / np.sqrt((object_corners[0][0] - object_corners[1][0])**2 + (object_corners[0][1] - object_corners[1][1])**2)

                    gripper_x = coordinate_conversion0*(gripper_corners[0][0] + gripper_corners[1][0]) / 2.0
                    gripper_y = coordinate_conversion0*(gripper_corners[0][1] + gripper_corners[1][1]) / 2.0

                    object_x = coordinate_conversion1*(object_corners[0][0] + object_corners[1][0]) / 2.0
                    object_y = coordinate_conversion1*(object_corners[0][1] + object_corners[1][1]) / 2.0

                    distance = np.sqrt((gripper_x - object_x)**2 + (gripper_y - object_y)**2)

                    angle = np.arctan((object_y - gripper_y) / (object_x - gripper_x))

                    t = rospy.get_time()
                    self.distances[f'{t}'] = [distance, angle]

                    cv2.putText(frame, "distance: " + str(round(distance,4)),
                        (10, 30), cv2.FONT_HERSHEY_DUPLEX,
                        0.5, (0, 255, 0), 1)
                    cv2.putText(frame, "angle: " + str(round(angle,4)),
                        (10, 60), cv2.FONT_HERSHEY_DUPLEX,
                        0.5, (0, 255, 0), 1)


                    # --------------------- Plot ArUco Markers --------------------#

                if ( rospy.get_time() - self.last_publish_time > (self.frame_rate*2) and ( len(list(corners)) > 0 ) ):

                    ids = ids.flatten()
                    
                    for (markerCorner, markerID) in zip(corners, ids):

                        corner = markerCorner.reshape((4, 2))
                        (topLeft, topRight, bottomRight, bottomLeft) = corner

                        topRight = (int(topRight[0]), int(topRight[1]))
                        bottomRight = (int(bottomRight[0]), int(bottomRight[1]))
                        bottomLeft = (int(bottomLeft[0]), int(bottomLeft[1]))
                        topLeft = (int(topLeft[0]), int(topLeft[1]))


                        # draw the bounding box of the ArUCo detection
                        cv2.line(frame, topLeft, topRight, (255, 0, 0), 2)
                        cv2.line(frame, topRight, bottomRight, (255, 0, 0), 2)
                        cv2.line(frame, bottomRight, bottomLeft, (255, 0, 0), 2)
                        cv2.line(frame, bottomLeft, topLeft, (255, 0, 0), 2)

                        cX = int((topLeft[0] + bottomRight[0]) / 2.0)
                        cY = int((topLeft[1] + bottomRight[1]) / 2.0)
                        cv2.circle(frame, (cX, cY), 4, (0, 0, 255), -1)

                        if ( markerID == 0 ):
                            cv2.putText(frame, "gripper",
                                (topLeft[0], topLeft[1] - 15), cv2.FONT_HERSHEY_DUPLEX,
                                0.5, (255, 0, 0), 1)
                        else:
                            cv2.putText(frame, "object",
                                (topLeft[0], topLeft[1] - 15), cv2.FONT_HERSHEY_DUPLEX,
                                0.5, (255, 0, 0), 1)
                                                        
                    # Print self.target_force in the frame
                    cv2.putText(frame, "Target force: " + str(round(self.target_force,3)), (10, 90), cv2.FONT_HERSHEY_DUPLEX, 0.5, (0, 255, 0), 1)
                    # print self.measured_fZ in the frame
                    cv2.putText(frame, "Measured force: " + str(round(np.sum(self.measured_fZ),3)), (10, 120), cv2.FONT_HERSHEY_DUPLEX, 0.5, (0, 255, 0), 1)
                    cv2.putText(frame, "Sensor 1 force: " + str(round(self.measured_fZ[0],3)), (10, 150), cv2.FONT_HERSHEY_DUPLEX, 0.5, (0, 255, 0), 1)
                    cv2.putText(frame, "Sensor 2 force: " + str(round(self.measured_fZ[1],3)), (10, 180), cv2.FONT_HERSHEY_DUPLEX, 0.5, (0, 255, 0), 1)
                    # cv2.putText(frame, "contactile force: " + str(self.contactile_tf), (10, 150), cv2.FONT_HERSHEY_DUPLEX, 0.5, (0, 255, 0), 1)

                    self.last_publish_time = rospy.get_time()

                    cv2.imshow("video", frame)
                    c = cv2.waitKey(1)
                    
                    # --------------------- Save data ---------------------#
                    t = rospy.get_time()
                    
                    self.images[f'{t}'] = frame

            elif ( not self.system_state) and ( rospy.get_time() - self.last_publish_time > PLOT_RATE): 
                cv2.imshow("video", frame)
                c = cv2.waitKey(1)
                self.last_publish_time = rospy.get_time()
            # self.writer.write(frame)
            self.last_frame_time = rospy.get_time()
        

    def shutdown(self, msg:Bool):
        if ( msg.data ) :
            rospy.loginfo('[%10s] shutdown registered'%self.node_string)
            rospy.signal_shutdown('[%10s] shutdown registered'%self.node_string)

    def rosrun(self):
        rospy.spin()

if __name__ == '__main__':
    try:
        detector_ros = RealSenseCamera()
    except rospy.ROSInterruptException:
        pass