#!/usr/bin/env  /usr/bin/python3
import rclpy
from rclpy.node import Node
from dynamixel_sdk_custom_interfaces.msg import   SetPosition
from dynamixel_sdk_custom_interfaces.srv import GetPosition
from fiborobotlab2.msg import Rstate
import serial
import time
import copy
import os
from ament_index_python.packages import get_package_share_directory
import threading

import simplejson
config_robot = simplejson.load(open(os.path.join(get_package_share_directory('fiborobotlab2'), 'config', 'robotname.json')))
robotname=config_robot["robotname"]
SetPosition_topic='set_position_{}'.format(robotname)
action_state_topic='action_state_{}'.format(robotname)
stainding_tipic='standing_status_{}'.format(robotname)

class State_Controll(Node):
    def __init__(self):
        super().__init__('State_Controll_{}'.format(robotname))
        self.publisher_ = self.create_publisher(Rstate, stainding_tipic, 10)     # CHANGE
        self.str_comport = '/dev/ttyACM0'
        self.baudrate = 115200
        self.robot_state = None
        self.Standing_status=None
        timer_period = 0.1 # seconds
        self.timer = self.create_timer(timer_period, self.green_button)
        print("$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$")
        print("Initial Locomotion : comport = " + self.str_comport + " , baudrate = " + str(self.baudrate))
        try:
            self.connect()
            print("Locomotion initial successfully + + +")
        except Exception as e:
            print(e)
            print("Locomotion initial fail!!")
        self.responsePacket_length = 7
        package = [255,255,0,4,3,2,140,106]
        self.serialDevice.write(package)
        time.sleep(0.01)
        self.responsePacket = self.serialDevice.read(self.serialDevice.inWaiting())    
    def connect(self):
        self.serialDevice = serial.Serial(port = self.str_comport, baudrate = self.baudrate, timeout=0)
    def green_button(self):
        responsePacket_length = 8
        package = [255,255,0,4,3,2,140,106]
        self.serialDevice.write(package)
        time.sleep(0.01)
        responsePacket = self.serialDevice.read(self.serialDevice.inWaiting())
        #print("responsePacket status=", responsePacket)
        #print("responsePacket length=", len(responsePacket))
        if(len(responsePacket) == responsePacket_length):
	    # if(responsePacket[5] == 0):
	    #     print("robot standing")
	    # else:
	    #     print("robot falling " + str(responsePacket[5]))
	    ##### return to server #####
            print(responsePacket)

def main(args=None):
    rclpy.init(args=args)
    Robotstate = State_Controll()
    #rclpy.spin(Detection)
    while rclpy.ok():
        rclpy.spin_once(Robotstate)
    Robotstate.destroy_node()
    rclpy.shutdown()
if __name__ == '__main__':
    main()
