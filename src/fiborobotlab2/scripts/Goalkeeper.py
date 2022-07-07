#!/usr/bin/env /usr/bin/python3

from requests import head
import rclpy
from rclpy.node import Node
import os
from fiborobotlab2.msg import Rstate
import simplejson
from ament_index_python.packages import get_package_share_directory
config_robot = simplejson.load(open(os.path.join(get_package_share_directory('fiborobotlab2'), 'config', 'robotname.json')))
robotname=config_robot["robotname"]
action_state_topic = 'action_state_{}'.format(robotname)
Head_position_topic = 'Head_position_{}'.format(robotname)

class Goalkeeper(Node):
    def __init__(self):
        super().__init__('Goalkeeper')
        self.publisher_ = self.create_publisher(Rstate, action_state_topic, 10)     # CHANGE
        self.subscription = self.create_subscription(Rstate, Head_position_topic, self.Side_walk,10)
        self.subscription # prevent unused variable warning
        self.head_pan=None
        self.head_tilt=None
        self.count=0
    def Side_walk(self, msg):
        self.head_pan = msg.pan_position
        print("head_pan : "+str(self.head_pan))
        self.head_tilt = msg.tilt_position
        if self.count<1:
            if(self.head_pan > 20):
                msg_action=Rstate()
                msg_action.action_state="left_walk"
                print("left_walk")
                self.publisher_.publish(msg_action)
                self.count+=1
            elif(self.head_pan < -20):
                msg_action=Rstate()
                msg_action.action_state="right_walk"
                self.publisher_.publish(msg_action)
                print("right_walk")
                self.count+=1
        if self.count ==1:
           self.head_pan=msg.pan_position
           self.head_tilt=msg.tilt_position
           self.count=0

def main(args=None):
    rclpy.init(args=args)
    goalkeeper = Goalkeeper()
    #rclpy.spin(Detection)
    while rclpy.ok():
        rclpy.spin_once(goalkeeper)
    goalkeeper.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()   

