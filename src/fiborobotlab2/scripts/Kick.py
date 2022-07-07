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
Headposition_topic='Head_position_{}'.format(robotname)
action_state_topic='action_state_{}'.format(robotname)
Ball_position_kick_topic='Ball_position_kick_{}'.format(robotname)
kick_topic='kick_status_{}'.format(robotname)

screen_size = [1280, 720]
class Kick(Node):
    def __init__(self):
        super().__init__('Kick_{}'.format(robotname))
        self.publisher_ = self.create_publisher(Rstate, action_state_topic, 10)     # CHANGE
        self.subscription = self.create_subscription(Rstate,Ball_position_kick_topic,self.check_object_x_position,10)
        self.publisher_2 = self.create_publisher(Rstate, kick_topic, 10)     # CHANGE
        self.subscription
        self.object_position_x=None
        self.object_position_y=None
        self.screen_center_x = screen_size[0]/2
        self.screen_center_y = screen_size[1]/2
        self.count=0
    def check_object_x_position(self,ballposition):
        self.object_position_x=ballposition.center_x
        self.object_position_y=ballposition.center_y
        x_ratio = (self.screen_center_x - self.object_position_x)/self.screen_center_x
        y_ratio = (self.screen_center_y - self.object_position_y)/self.screen_center_y
        if(x_ratio != None):
            print(y_ratio)
            if(y_ratio < 0.2):
                if(x_ratio > 0 and x_ratio < 0.8):
                    msg_action=Rstate()
                    msg_action.action_state="left_kick"
                    self.publisher_.publish(msg_action)

                    print("Left Kick")

                elif(x_ratio < 0 and x_ratio > -0.8):
                    msg_action=Rstate()
                    msg_action.action_state="right_kick"
                    self.publisher_.publish(msg_action)
                    print("Right Kick")

                else:
                    msg_kick_status=Rstate()
                    msg_kick_status.kick_status="dont'_kick"
                    self.publisher_2.publish(msg_kick_status)
            else:
                msg_kick_status=Rstate()
                msg_kick_status.kick_status="dont'_kick"
                self.publisher_2.publish(msg_kick_status)
def main(args=None):
    rclpy.init(args=args)
    Kick_state = Kick()
    #rclpy.spin(Detection)
    while rclpy.ok():
        rclpy.spin_once(Kick_state)
    Kick_state.destroy_node()
    rclpy.shutdown()
if __name__ == '__main__':
    main()              
           