#!/home/pannatorn/anaconda3/bin/python

import rclpy
from rclpy.node import Node
from dynamixel_sdk_custom_interfaces.msg import   SetPosition
from dynamixel_sdk_custom_interfaces.srv import GetPosition
from fiborobotlab2.msg import Rstate

import time
import copy
class confirmed_node(Node):
    def __init__(self):
        super().__init__('Detection_state')
        self.publisher_ = self.create_publisher(Rstate, 'detection_state', 10)
        self.subscription = self.create_subscription(Rstate,'detection_check',self.detection_state_sub)
        timer_period = 1/30  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)
        self.detection_state="Detect_ball"
    def detection_state_sub(self,msg):
        self.detection_state=msg.detection_state
    def timer_callback(self):
        msg = Rstate()
        msg.detection_state = self.detection_state
        self.publisher_.publish(msg)
        self.get_logger().info('Publishing: "%s"' % msg.detection_state)
def main(args=None):
    rclpy.init(args=args)
    conf = confirmed_node()
    #rclpy.spin(Detection)
    while rclpy.ok():
        rclpy.spin_once(conf)
    conf.destroy_node()
    rclpy.shutdown()
if __name__ == '__main__':
    main()                   