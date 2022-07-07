#!/usr/bin/env  /usr/bin/python3

import rclpy
from rclpy.node import Node

from fiborobotlab2.msg import Rstate
import os
import simplejson
from ament_index_python.packages import get_package_share_directory

config_robot = simplejson.load(open(os.path.join(get_package_share_directory('fiborobotlab2'), 'config', 'robotname.json')))
robotname=config_robot["robotname"]

Ball_position_topic='Ball_position_{}'.format(robotname)
action_state_topic = 'action_state_{}'.format(robotname)
# width, height = 1280, 720
# X_bound = obj_list[0].bbox.xmax - obj_list[0].bbox.xmin
# Y_bound = obj_list[0].bbox.ymax - obj_list[0].bbox.ymin
class Goalkeeper(Node):

    def __init__(self):
        super().__init__('Goalkeeper2_{}'.format(robotname))
        self.publisher_ = self.create_publisher(Rstate, action_state_topic, 10)
        self.subscription = self.create_subscription(Rstate,Ball_position_topic,self.Goal_play,10)
        self.subscription
        self.center_x = None
        self.center_y = None
        self.x_line = None
        self.y_line = None
        self.count = 0
        self.area = 0
        self.count_action = 0

    def Goal_play(self, msg):
        self.center_x = msg.center_x
        self.center_y = msg.center_y
        self.x_line = msg.x_line
        self.y_line = msg.y_line
        self.area = (self.x_line) * (self.y_line) 
        # self.count_action = 0
        if (self.center_x > 1070) & (self.area > (100*120)):# & (velocity > value):        # (100*120) is area of ball when ball near Goalkeeper.
            if self.count_action == 0 :
                msg_action=Rstate()
                msg_action.action_state="right_save"
                print("right_save")
                self.publisher_.publish(msg_action)

                msg_action=Rstate()
                msg_action.action_state="turn_right"
                print("turn_right")
                self.publisher_.publish(msg_action)
                self.count_action +=1
            self.count_action = 0
        if (self.center_x < 160) & (self.area> (100*120)):# & (velocity > value):
            if self.count_action == 0 :
                msg_action=Rstate()
                msg_action.action_state="left_save"
                print("left_save")
                self.publisher_.publish(msg_action)

                msg_action=Rstate()
                msg_action.action_state="turn_left"
                print("turn_left")
                self.publisher_.publish(msg_action)
                self.count_action +=1
            self.count_action = 0
        elif (self.area > (100*120)):# & (velocity < value):
            print("follow_ball and kick")
        elif (self.center_x > 1070) & (self.area < (100*120)):
            msg_action=Rstate()
            msg_action.action_state="walk_right_slide"
            print("walk_right_slide")
            self.publisher_.publish(msg_action)
        elif (self.center_x < 160) & (self.area < (100*120)):
            msg_action=Rstate()
            msg_action.action_state="walk_left_slide"
            print("walk_left_slide")
            self.publisher_.publish(msg_action)
        else:
            msg_action=Rstate()
            msg_action.action_state="stop"
            print("stop")
            self.publisher_.publish(msg_action)


def main(args=None):
    rclpy.init(args=args)

    goalkeeper = Goalkeeper()

    while rclpy.ok():
        rclpy.spin_once(goalkeeper)

    goalkeeper.destroy_node()
    rclpy.shutdown()
if __name__ == '__main__':
    main()
