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
import threading
from ament_index_python.packages import get_package_share_directory
import simplejson
config_robot = simplejson.load(open(os.path.join(get_package_share_directory('fiborobotlab2'), 'config', 'robotname.json')))
robotname=config_robot["robotname"]
Headposition_topic='Head_position_{}'.format(robotname)
action_state_topic='action_state_{}'.format(robotname)
kick_topic='kick_status_{}'.format(robotname)
detection_state_topic = 'detection_state_{}'.format(robotname)

class Follow_ball(Node):
    def __init__(self):
        super().__init__('Fllow_ball_{}'.format(robotname))
        self.publisher_ = self.create_publisher(Rstate, action_state_topic, 10)     # CHANGE
        self.publisher_2 = self.create_publisher(Rstate, detection_state_topic, 10)     # CHANGE

        #self.publisher_2 = self.create_publisher(Rstate, kick_topic, 10)     # CHANGE
        self.subscription = self.create_subscription(Rstate,Headposition_topic,self.Follow_ball,10)
        self.subscription
        self.head_pan=None
        self.head_tilt=None
        self.count=0
        self.count_stop=0

    def Follow_ball(self,msg):
        # self.Follow_ball=msg.follow_ball_status
        self.head_pan=msg.pan_position
        print("head_pan : "+str(self.head_pan))
        self.head_tilt=msg.tilt_position
        if self.count<1:
            if(self.head_pan > 20):
                msg_action=Rstate()
                msg_action.action_state="turn_left"
                print("turn_left")
                self.publisher_.publish(msg_action)
                self.count_stop=0
                self.count+=1
            elif(self.head_pan < -20):
                msg_action=Rstate()
                msg_action.action_state="turn_right"
                self.publisher_.publish(msg_action)
                print("turn_right")   
                self.count_stop=0 
                self.count+=1
       
            else:
                print(self.head_tilt)
                if(self.head_tilt < 25):
                    msg_action=Rstate()
                    msg_action.action_state="forward_berserk"
                    self.publisher_.publish(msg_action)
                    print("forward_berserk") 
                    self.count_stop=0
                    self.count+=1
                elif(self.head_tilt < 40):
                    msg_action=Rstate()
                    msg_action.action_state="forward_range"
                    self.publisher_.publish(msg_action)
                    print("forward_range") 
                    self.count_stop=0
                    self.count+=1
                elif(self.head_tilt < 50):
                    msg_action=Rstate()
                    msg_action.action_state="forward_almost_normal"
                    self.publisher_.publish(msg_action)
                    print("forward_almost_normal") 
                    self.count_stop=0
                    self.count+=1
                elif(self.head_tilt < 60):
                    msg_action=Rstate()
                    msg_action.action_state="forward_normal"
                    self.publisher_.publish(msg_action)
                    print("forward_normal") 
                    self.count_stop=0
                    self.count+=1
                 
                else:
                    msg_action=Rstate()
                    msg_action.action_state="stop"
                    self.publisher_.publish(msg_action)
                    print("stop")
                    self.count_stop+=1
                    if self.count_stop > 30:
                        # msg_kick_status=Rstate()
                        # msg_kick_status.kick_status="kick"
                        msg_detection_state=Rstate()
                        msg_detection_state.detection_state='detect_goal'
                        self.publisher_2.publish(msg_detection_state)
                        print("scall_goal")
                        self.count_stop=0
                    self.count+=1

        if self.count ==1:
           self.head_pan=msg.pan_position
           self.head_tilt=msg.tilt_position
           self.count=0

    
def main(args=None):
    rclpy.init(args=args)
    follow_ball = Follow_ball()
    #rclpy.spin(Detection)
    while rclpy.ok():
        rclpy.spin_once(follow_ball)
    follow_ball.destroy_node()
    rclpy.shutdown()
if __name__ == '__main__':
    main()              
