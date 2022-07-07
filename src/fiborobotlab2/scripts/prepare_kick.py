#!/usr/bin/env  /usr/bin/python3

from multiprocessing.spawn import prepare
import rclpy
from rclpy.node import Node
from dynamixel_sdk_custom_interfaces.msg import   SetPosition
from dynamixel_sdk_custom_interfaces.srv import GetPosition
from fiborobotlab2.msg import Rstate
import serial
import time
import os 
from ament_index_python.packages import get_package_share_directory
import simplejson
config_robot = simplejson.load(open(os.path.join(get_package_share_directory('fiborobotlab2'), 'config', 'robotname.json')))
robotname=config_robot["robotname"]
Headposition_topic='Head_position_{}'.format(robotname)
action_state_topic='action_state_{}'.format(robotname)
goal_position_topic='goal_position_{}'.format(robotname)
kick_topic='kick_status_{}'.format(robotname)
detection_state_topic = 'detection_state_{}'.format(robotname)

class Prepare_Kick(Node):
    def __init__(self):
        super().__init__('Prepare_Kick_{}'.format(robotname))
        self.publisher_ = self.create_publisher(Rstate, action_state_topic, 10)     # CHANGE
        self.subscription = self.create_subscription(Rstate,goal_position_topic,self.Goal_curve,10)
        self.publisher_1 = self.create_publisher(Rstate, detection_state_topic, 10)     # CHANGE
        self.publisher_2 = self.create_publisher(Rstate, kick_topic, 10)     # CHANGE
        self.subscription
        self.head_pan=None
        self.count=0
        self.stop_count=0
    
    def Goal_curve(self,msg):
        # self.Follow_ball=msg.follow_ball_status
        self.head_pan=msg.pan_position
        #print("head_pan : "+str(self.head_pan))
        print(self.head_pan)
        if self.count<1:
            if(self.head_pan > 20):
                msg_action=Rstate()
                msg_action.action_state="right_curve"
                print("right_curve")
                self.publisher_.publish(msg_action)
                self.stop_count=0
                self.count+=1
            elif(self.head_pan < -20):
                msg_action=Rstate()
                msg_action.action_state="left_curve"
                self.publisher_.publish(msg_action)
                print("left_curve")  
                self.stop_count=0  
                self.count+=1
            else:
                msg_action=Rstate()
                msg_action.action_state="stop"
                self.publisher_.publish(msg_action)
                print("stop")
                if self.stop_count > 30:
                    msg_set_head_status=Rstate()
                    msg_set_head_status.kick_status="set_head_kick"
                    self.publisher_2.publish(msg_set_head_status)
                    self.stop_count=0
                self.stop_count+=1
                self.count+=1
        if self.count ==1:
           self.head_pan=msg.pan_position
           self.count=0
def main(args=None):
    rclpy.init(args=args)
    prepare_Kick = Prepare_Kick()
    #rclpy.spin(Detection)
    while rclpy.ok():
        rclpy.spin_once(prepare_Kick)
    prepare_Kick.destroy_node()
    rclpy.shutdown()
if __name__ == '__main__':
    main()        

       