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
        super().__init__('State_Controll')
        self.subscription = self.create_subscription(Rstate,'prepare',self.State_manager,10)
        self.publisher_2 = self.create_publisher(Rstate, 'Robot_state', 10)     # CHANGE
        self.prev=None
        self.state=None
    def State_manager(self,msg):
        msg2=Rstate()
        self.prev =msg.robot_state
        if self.state == "scanball":
            self.get_logger().info('robot_state %s ' % (msg.robot_state))
            prev=time.time()
            while(time.time()-prev<5):
                self.state =msg.robot_state
        if self.prev == self.state:
            msg2.robot_state=self.state
            self.get_logger().info('robotstate %s ' % (msg2.robot_state))
            self.publisher_2.publish(msg2)

 
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