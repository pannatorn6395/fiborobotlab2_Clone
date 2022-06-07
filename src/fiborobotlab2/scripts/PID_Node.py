#!/home/pannatorn/anaconda3/bin/python
import rclpy
from rclpy.node import Node
from dynamixel_sdk_custom_interfaces.msg import   SetPosition
from dynamixel_sdk_custom_interfaces.srv import GetPosition
from fiborobotlab2.msg import BBox

import time


class PID_node(Node):
    def __init__(self):
        super().__init__('PID_node')
        self.publisher_ = self.create_publisher(SetPosition, 'set_position', 10)     # CHANGE
        self.cli = self.create_client(GetPosition, 'get_position')
        self.subscription = self.create_subscription(BBox,'Ball_position',self.Calculate_PID,10)
        # timer_period = 0.5
        # self.timer = self.create_timer(timer_period, self.timer_callback)
        # self.position = 0
        # self.ID = 1
        # self.req = GetPosition.Request()
        # self.count=0
        self.subscription
        self.center_x=0
        self.center_y=0
        self.pan = 41
        self.tilt=42
        self.req = GetPosition.Request()
    def Calculate_PID(self,msg):
        self.get_logger().info('subscript center x: %d center y: %d ' % (msg.center_x,msg.center_y))
        self.center_x=msg.center_x
        self.center_y=msg.center_y
        self.send_request()
        self.send_request2()

        
    def send_request(self): 
        self.req.id = 41   
        self.future = self.cli.call_async(self.req)
    def send_request2(self): 
        self.req.id = 42   
        self.future_2 = self.cli.call_async(self.req)

def main(args=None):
    rclpy.init(args=args)
    PID = PID_node()
    while rclpy.ok():
        rclpy.spin_once(PID)
        if PID.future.done() and PID.future_2.done():
                try:
                    response = PID.future.result()
                    response2 = PID.future_2.result()
                except Exception as e:
                    PID.get_logger().info(
                        'Service call failed %r' % (e,))
                else:
                    presentposition=response.position
                    PID.get_logger().info(
                        'Present_position : %d' %
                        ( response.position))
                    PID.get_logger().info(
                        'Present_position2 : %d' %
                        ( response2.position))
        
        # motor_position_x
        # print(PID.center_x)
        # motor_position_next_x = motor_position_x + self.pid_pan.update(self.screen_center_x, object_position_x)
        # if (motor_position_next_x > pan_angle_limit[1]): motor_position_next_x = pan_angle_limit[1]
        # elif (motor_position_next_x < pan_angle_limit[0]): motor_position_next_x = pan_angle_limit[0]
        # motor_position_next_y = motor_position_y + (-1)*self.pid_tilt.update(self.screen_center_y, object_position_y)
        # if (motor_position_next_y > tilt_angle_limit[1]): motor_position_next_y = tilt_angle_limit[1]
        # elif (motor_position_next_y < tilt_angle_limit[0]): motor_position_next_y = tilt_angle_limit[0]

if __name__ == '__main__':
    main()
