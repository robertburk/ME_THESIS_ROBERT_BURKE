# UCD_robot_ws
Catkin workspace for the UCD_robot 

| Component | Version / Spec |
| ------------- | ------------- |
| PC  | Acer Aspire V3-574g  |
| SDD  | 1TB  |
| RAM | 8GB |
| Linux Distribution | Ubuntu 20.04 Focal Fossa |
| Kernel Version | 5.15-rt |
| ROS Distribution | ROS 1 Noetic |

## Components
This project is a ROS1 workspace built to interface with a UR5e Universal Robots cobot (https://www.universal-robots.com/products/ur5-robot/), a gripper driven by a Dynamixel MX-28 servo motor (https://emanual.robotis.com/docs/en/dxl/mx/mx-28/), Contactile tactile sensors (https://contactile.com/), and an Intel RealSense D435 camera (https://www.intelrealsense.com/depth-camera-d435/)

## Installing Packages

### Installing the Dynamixel SDK

GitHub link: https://github.com/ROBOTIS-GIT/DynamixelSDK </br>
Robotis Manual for Dynamixel SDK with installation guide: https://emanual.robotis.com/docs/en/software/dynamixel/dynamixel_sdk/overview/

### Installing the Universal Robots ROS drivers
GitHub link with full installation manual: https://github.com/UniversalRobots/Universal_Robots_ROS_Driver </br>

**NOTE** To operate the UR5e using this driver, you must be running a realtime kernel version of your Linux distribution, installation instructions are also in the link above

### Installation of pyrealsense2

Fortunately, the Realsense SDK is not necessary here, instead the package [pyrealsense2](https://pypi.org/project/pyrealsense2/) is used to extract the intrinsic matrices of the camera for initialisation, and [openCV](https://pypi.org/project/opencv-python/) is used thereafter



