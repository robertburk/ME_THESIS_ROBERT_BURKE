# -*- coding: utf-8 -*-
"""
Created on Mon May 30 15:22:03 2022

@author: Qiang
"""

import dynamixel_sdk
import math

'''
Original information:
Firmware version = 41
ID = ~
Baud Rate = 34 = 57600 bps
Return delay time = 250 = 500msec
CW angle limit = 0 = 0°
CWW angle limit = 4095 = 360°
'''


ADDR_MX_TORQUE_ENABLE      = 24             # Control table address is different in Dynamixel model
ADDR_MX_GOAL_POSITION      = 30
ADDR_MX_PRESENT_POSITION   = 36
ADDR_MX_POS_LIMIT = 8
ADDR_RETURN_DELAY_TIME = 5
ADDR_MX_MOVING = 46
ADDR_MX_VOLTAGE = 42
ADDR_MX_LOAD = 40
ADDR_MX_SPEED = 38
ADDR_MX_MOVING_SPEED = 32
ADDR_MX_CW_LIMIT = 6
ADDR_MX_CCW_LIMIT = 8
ADDR_MX_MAX_TORQUE = 14
ADDR_MX_TORQUE_LIMIT = 34
ADDR_MX_RES_DIVIDER = 22
ADDR_MX_ACCELERATE = 73
ADDR_MX_MULTIRUN_OFFSET = 20

BAUDRATE                   = 2000000

PROTOCOL_VERSION = 1.0

# Port operating

def openport(PORT_NUM):
    # open the port for communication
    '''port : port number for connecting the usb on your PC, you can check it
        in the device manager.
    example: "COM5"'''
    
    port = dynamixel_sdk.PortHandler(PORT_NUM)
    if not port.openPort():
        print("[RoboticSer] Failed to open the port")
    if port.setBaudRate(BAUDRATE):
        return port
    else:
        print("[RoboticSer] Failed to change the baudrate")
    # return port

def openpacket():
    # open the port for sending data
    packet = dynamixel_sdk.Protocol1PacketHandler()
    if packet:
        return packet
    else:
        print("[RoboticSer] Fail to open the Packet")

class Robotis_Servo():
    def __init__(self, port, packet, servo_id):
        '''
        Parameters
        ----------
        port: the instantiated object of port from the above openport
        packet : the instantiated object of packet from the above openpacket
        servo_id : servo's ID, you can check it from dynamixel wizard.
        '''
        
        self.return_delay = 0
        self.servo_id = servo_id
        self.settings = {
            'home_encoder': 0x7FF,
            'max_encoder': 0xFFF,
            'rad_per_enc': math.radians(360.0) / 0xFFF,
            'max_ang': math.radians(180),
            'min_ang': math.radians(-180),
            'flipped': False,
            'max_speed': math.radians(500)
        }
        self.port = port
        self.packet = packet

    # The following function can be applied to check the status of the servo(s)
    def read_current_pos(self):
        dxl_present_position, dxl_result, dxl_error = self.packet.read2ByteTxRx(self.port, self.servo_id, ADDR_MX_PRESENT_POSITION)
        if self.check_com(dxl_result, dxl_error):
            return dxl_present_position
        else:
            print('Some problem with communication during read goal')
            return None

    def read_goal_position(self):
        dxl_goal_position, dxl_result, dxl_error = self.packet.read2ByteTxRx(self.port, self.servo_id,ADDR_MX_GOAL_POSITION)
        if self.check_com(dxl_result, dxl_error):
            return dxl_goal_position
        else:
            print('Some problem with communication during read goal')
            return None

    def is_moving(self):
        # check if the servo is moving
        dxl_is_moving, dxl_result, dxl_error = self.packet.read1ByteTxRx(self.port, self.servo_id,ADDR_MX_MOVING)
        if self.check_com(dxl_result, dxl_error):
            return dxl_is_moving

    def read_voltage(self):
        dxl_voltage, dxl_result, dxl_error = self.packet.read1ByteTxRx(self.port, self.servo_id, ADDR_MX_VOLTAGE)
        if self.check_com(dxl_result, dxl_error):
            return dxl_voltage/10.

    def read_load(self):
        dxl_load, dxl_result, dxl_error = self.packet.read2ByteTxRx(self.port, self.servo_id, ADDR_MX_LOAD)
        if self.check_com(dxl_result, dxl_error):
            return dxl_load
        else:
            print('Some problem with communication during reading load')
            return -1

    def read_goal(self):
        dxl_load, dxl_result, dxl_error = self.packet.read2ByteTxRx(self.port, self.servo_id, ADDR_MX_GOAL_POSITION)
        if self.check_com(dxl_result, dxl_error):
            return dxl_load
        else:
            print('Some problem with communication during reading goal')
            return None

    def read_speed(self):
        dxl_speed, dxl_result, dxl_error = self.packet.read2ByteTxRx(self.port, self.servo_id, ADDR_MX_SPEED)
        if self.check_com(dxl_result, dxl_error):
            return dxl_speed
        else:
            print('Some problem with communication during reading speed')
            return None

    # The following function can be applied for movement or change the parameters of movement
    def moveit(self, position, mode):
        # go to the position
        success = False
        # mode = 'multiturn'
        if mode=='wheel':
            print("goto function cannot be applied on wheel mode, please change to another mode")
        elif mode=='joint':
            cw_limit = self.check_cw_limit()
            ccw_limit = self.check_ccw_limit()
            if position>max(cw_limit,ccw_limit) or position<min(cw_limit,ccw_limit):
                print("The input position value is invalid, which is out of the range of cw and ccw angle limit")
            else:
                dxl_result, dxl_error = self.packet.write2ByteTxRx(self.port, self.servo_id, ADDR_MX_GOAL_POSITION,position)
                self.check_com(dxl_result, dxl_error)
        elif mode=='multiturn':
            if position>28672 or position<-28672:
                print("The input position is invalid, which is out of range, valid range is -28672~28672")
            else:
                dxl_result, dxl_error = self.packet.write2ByteTxRx(self.port, self.servo_id, ADDR_MX_GOAL_POSITION, position)
                if (self.check_com(dxl_result, dxl_error)):
                    success = True
        return success

    def enable_torque(self):
        dxl_result, dxl_error = self.packet.write1ByteTxRx(self.port, self.servo_id, ADDR_MX_TORQUE_ENABLE,1)
        if not self.check_com(dxl_result, dxl_error):
            raise RuntimeError("Unable to enable torque")
        else:
            return True

    def disable_torque(self):
        dxl_result, dxl_error = self.packet.write1ByteTxRx(self.port, self.servo_id, ADDR_MX_TORQUE_ENABLE,0)
        if not self.check_com(dxl_result, dxl_error):
            raise RuntimeError("Unable to disable torque")

    def set_return_delay_time(self,value):
        dxl_result, dxl_error = self.packet.write1ByteTxRx(self.port, self.servo_id, ADDR_RETURN_DELAY_TIME,value)
        if not self.check_com(dxl_result, dxl_error):
            raise RuntimeError("Unable to change return delay time")

    def wheel_set_speed(self, direction, speed):
        mode = self.check_move_mode()
        if mode=="wheel":
            if speed<0 or speed>2047:
                raise RuntimeError("Input speed is not valid, which is out of range. The valid range is [0,1023]")
            elif direction == "CW":
                speed += 1024
                dxl_result, dxl_error = self.packet.write2ByteTxRx(self.port, self.servo_id, ADDR_MX_MOVING_SPEED, speed)
                self.check_com(dxl_result, dxl_error)
            elif direction == "CCW":
                dxl_result, dxl_error = self.packet.write2ByteTxRx(self.port, self.servo_id, ADDR_MX_MOVING_SPEED, speed)
                self.check_com(dxl_result, dxl_error)
        else:
            raise RuntimeError("Current moving mode is not wheel mode, please change the mode to wheel mode and then use this function")

    def joint_set_speed(self, speed):
        mode = self.check_move_mode()
        if mode=="joint":
            if speed<0 or speed>1023:
                raise RuntimeError("Input speed is not valid, which is out of range. The valid range is [0,1023]")
            else:
                dxl_result, dxl_error = self.packet.write2ByteTxRx(self.port, self.servo_id, ADDR_MX_MOVING_SPEED, speed)
                self.check_com(dxl_result, dxl_error)
        else:
            raise RuntimeError("Current moving mode is not joint mode, please change the mode to wheel mode and then use this function")

    def multiturn_set_speed(self, speed):
        mode = self.check_move_mode()
        if mode=="multiturn":
            if speed<0 or speed>1023:
                raise RuntimeError("Input speed is not valid, which is out of range. The valid range is [0,1023]")
            else:
                dxl_result, dxl_error = self.packet.write2ByteTxRx(self.port, self.servo_id, ADDR_MX_MOVING_SPEED, speed)
                if not self.check_com(dxl_result, dxl_error):
                    raise RuntimeError('Some problems with communication during setting speed')
        else:
            raise RuntimeError("Current moving mode is not joint mode, please change the mode to wheel mode and then use this function")
            
    def set_accel(self, accel=0):
        if accel<0 or accel>254:
            raise RuntimeError("Input speed is not valid, which is out of range. The valid range is [0,1023]")
        else:
            dxl_result, dxl_error = self.packet.write1ByteTxRx(self.port, self.servo_id, ADDR_MX_ACCELERATE, accel)
            if not self.check_com(dxl_result, dxl_error):
                raise RuntimeError('Some problems with communication during setting acceleration')

    def set_torque(self,torque):
        if torque>1023 or torque<0:
            raise RuntimeError("Invalid input torque value")
        else:
            dxl_result, dxl_error = self.packet.write2ByteTxRx(self.port, self.servo_id, ADDR_MX_TORQUE_LIMIT, torque)
            if not self.check_com(dxl_result, dxl_error):
                raise RuntimeError('Some problems with communication during setting torque')
            # dxl_result, dxl_error = self.packet.write2ByteTxRx(self.port, self.servo_id, ADDR_MX_MAX_TORQUE, torque)
            # self.check_com(dxl_result, dxl_error)
            
    def set_resolution(self, resolution=1):
        if resolution<1 or resolution>4:
            raise RuntimeError("Input speed is not valid, which is out of range. The valid range is int[1,4]")
        else:
            dxl_result, dxl_error = self.packet.write1ByteTxRx(self.port, self.servo_id, ADDR_MX_RES_DIVIDER, resolution)
            if not self.check_com(dxl_result, dxl_error):
                raise RuntimeError('Some problems with communication during setting resolution')

    # The following functions can be applied to Change the movement mode and working range
    def set_cw_limit(self,num):
        dxl_result, dxl_error = self.packet.write2ByteTxRx(self.port, self.servo_id, ADDR_MX_CW_LIMIT, int(num))
        if not self.check_com(dxl_result, dxl_error):
            raise RuntimeError("Fail to set CW limit to position %d" % int(num))

    def set_ccw_limit(self, num):
        dxl_result, dxl_error = self.packet.write2ByteTxRx(self.port, self.servo_id, ADDR_MX_CCW_LIMIT, int(num))
        if not self.check_com(dxl_result, dxl_error):
            raise RuntimeError("Fail to set CW limit to position %d" % int(num))

    def init_joint_mode(self, cw_limit, cww_limit):
        self.set_cw_limit(cw_limit)
        self.set_ccw_limit(cww_limit)

    def init_wheel_mode(self):
        self.set_cw_limit(0)
        self.set_ccw_limit(0)

    def init_multiturn_mode(self):
        self.set_cw_limit(4095)
        self.set_ccw_limit(4095)

    def reset_move_mode(self):
        self.set_cw_limit(0)
        self.set_ccw_limit(4095)

    # The following functions can be applied to check the movement mode and working range
    def check_cw_limit(self):
        cw_limit, dxl_result, dxl_error = self.packet.read2ByteTxRx(self.port, self.servo_id, ADDR_MX_CW_LIMIT)
        if self.check_com(dxl_result, dxl_error):
            return cw_limit
        else:
            raise RuntimeError('Some problems with communication during checking limit')

    def check_ccw_limit(self):
        ccw_limit, dxl_result, dxl_error = self.packet.read2ByteTxRx(self.port, self.servo_id, ADDR_MX_CCW_LIMIT)
        if self.check_com(dxl_result, dxl_error):
            return ccw_limit
        else:
            raise RuntimeError('Some problems with communication during checking limit')

    def check_move_mode(self):
        cw_limit = self.check_cw_limit()
        ccw_limit = self.check_ccw_limit()

        if cw_limit==0 and ccw_limit==0:
            return "wheel"

        if cw_limit>0 and cw_limit<4095 and ccw_limit>0 and ccw_limit<4095:
            return "joint"

        if cw_limit==4095 and ccw_limit==4095:
            return "multiturn"

    def check_com(self,result,error):
        if result != dynamixel_sdk.COMM_SUCCESS:
            print("%s" % self.packet.getTxRxResult(result))
            return 0
        elif error != 0:
            print("%s" % self.packet.getRxPacketError(error))
            return 0
        else:
            return 1





