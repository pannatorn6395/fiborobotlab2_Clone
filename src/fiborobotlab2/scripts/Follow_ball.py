#!/home/pannatorn/anaconda3/bin/python

import rclpy
from rclpy.node import Node
from dynamixel_sdk_custom_interfaces.msg import   SetPosition
from dynamixel_sdk_custom_interfaces.srv import GetPosition
from fiborobotlab2.msg import Rstate
import serial
import time
import copy

import threading
class Follow_ball(Node):
    def __init__(self):
        super().__init__('Fllow_ball')
        self.publisher_ = self.create_publisher(Rstate, 'action_state', 10)     # CHANGE
        self.subscription = self.create_subscription(Rstate,'Head_position',self.Follow_ball,10)
        self.subscription
        self.head_pan=None
        self.head_tilt=None
        self.count=0

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
                self.count+=1
            elif(self.head_pan < -20):
                msg_action=Rstate()
                msg_action.action_state="turn_right"
                self.publisher_.publish(msg_action)
                print("turn_right")    
                self.count+=1
       
            else:
                print(self.head_tilt)
                if(self.head_tilt < 55):
                    msg_action=Rstate()
                    msg_action.action_state="forward"
                    self.publisher_.publish(msg_action)
                    print("forward") 
                    self.count+=1
                 
                else:
                    msg_action=Rstate()
                    msg_action.action_state="stop"
                    self.publisher_.publish(msg_action)
                    print("stop")
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