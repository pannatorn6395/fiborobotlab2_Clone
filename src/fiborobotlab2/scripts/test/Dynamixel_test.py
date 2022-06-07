#!/home/pannatorn/anaconda3/bin/python
import rclpy
from rclpy.node import Node
from dynamixel_sdk_custom_interfaces.msg import   SetPosition
from dynamixel_sdk_custom_interfaces.srv import GetPosition
import time
global presentposition 
presentposition = None

class Dynamixel_pub(Node):

    def __init__(self):
        super().__init__('Dynamixel_pub')
        self.publisher_ = self.create_publisher(SetPosition, 'set_position', 10)     # CHANGE
        self.cli = self.create_client(GetPosition, 'get_position')
        timer_period = 0.5
        self.timer = self.create_timer(timer_period, self.timer_callback)
        self.position = 2048
        self.pan = 41
        self.tilt=42
        self.req = GetPosition.Request()
        self.count=0



    def timer_callback(self):
        msg = SetPosition()                                           # CHANGE
        msg.position = self.position
        msg.id=self.pan                                 # CHANGE
        self.publisher_.publish(msg)
        self.get_logger().info('Publishing pan: "%d"' % msg.position)  # CHANGE
        msg.id=self.tilt                                # CHANGE
        self.publisher_.publish(msg)
        self.get_logger().info('Publishing tilt: "%d"' % msg.position)  # CHANGE
        self.position += 10
        time.sleep(0.1)
        self.send_request()
        if presentposition != None:
            print(presentposition)
        self.count+=1


    
    def send_request(self):
        self.req.id = 41
        self.future = self.cli.call_async(self.req)

def main(args=None):
    rclpy.init(args=args)
    print("11")
    Dynamixel_publisher = Dynamixel_pub()
    while rclpy.ok():
        rclpy.spin_once(Dynamixel_publisher)
        if Dynamixel_publisher.count <100 :
            if Dynamixel_publisher.future.done():
                try:
                    response = Dynamixel_publisher.future.result()
                except Exception as e:
                    Dynamixel_publisher.get_logger().info(
                        'Service call failed %r' % (e,))
                else:
                    presentposition=response.position
                    Dynamixel_publisher.get_logger().info(
                        'Present_position : %d' %
                        ( response.position))

    Dynamixel_publisher.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
