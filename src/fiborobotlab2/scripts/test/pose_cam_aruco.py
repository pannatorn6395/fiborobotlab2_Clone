#!/home/pannatorn/anaconda3/bin/python
import rclpy
from rclpy.node import Node
from dynamixel_sdk_custom_interfaces.msg import   SetPosition
from dynamixel_sdk_custom_interfaces.srv import GetPosition
import time
global presentposition 
presentposition = None
pan_angle_limit = [-90, 90]
tilt_angle_limit = [-45, 70]
class pose_cam_aruco(Node):

    def __init__(self):
        super().__init__('pose_cam_aruco')
        self.publisher_ = self.create_publisher(SetPosition, 'set_position', 10)     # CHANGE
        self.cli = self.create_client(GetPosition, 'get_position')
        timer_period = 0.5
        self.timer = self.create_timer(timer_period, self.timer_callback)
        self.position = -90
        self.position2=-45
        self.pan = 41
        self.tilt=42
        self.req = GetPosition.Request()
        self.count=0

    def convertAngleDegToMotorValue(self,angle_deg):
        motor_value = (angle_deg + 180) * 4096 / 360
        return int(motor_value)

    def timer_callback(self):
        if self.position2 <tilt_angle_limit[1]:
            msg = SetPosition()                                           # CHANGE
            msg.id=self.pan                                 # CHANGE
            if (self.position > pan_angle_limit[1]): 
                self.position = pan_angle_limit[1]
                self.position2 += 5
                self.position=-90
            elif (self.position < pan_angle_limit[0]): 
                self.position = pan_angle_limit[0]
            msg.position = self.convertAngleDegToMotorValue(self.position)
            self.get_logger().info('Publishing pan: "%d"' % msg.position)  # CHANGE
            self.publisher_.publish(msg)
            #----------------------------------------------------------------
            msg.id=self.tilt
            if (self.position2 > tilt_angle_limit[1]): 
                self.position2 = tilt_angle_limit[1]
            elif (self.position2 < tilt_angle_limit[0]): 
                self.position2 = tilt_angle_limit[0]
            msg.position = self.convertAngleDegToMotorValue(self.position2)                                # CHANGE
            self.publisher_.publish(msg)
            self.get_logger().info('Publishing tilt: "%d"' % msg.position)  # CHANGE
            self.position += 5
        else:
            while(self.count<1):
                msg.id=self.pan                                 # CHANGE
                self.position=0
                msg.position = self.convertAngleDegToMotorValue(self.position)
                self.get_logger().info('Publishing pan: "%d"' % msg.position)  # CHANGE
                self.publisher_.publish(msg)
                
                msg.id=self.tilt
                self.position2=55
                msg.position = self.convertAngleDegToMotorValue(self.position2)
                self.get_logger().info('Publishing pan: "%d"' % msg.position)  # CHANGE
                self.publisher_.publish(msg)
                self.count+=1

def main(args=None):
    rclpy.init(args=args)
    print("11")
    pose = pose_cam_aruco()
    rclpy.spin(pose)

    pose.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
