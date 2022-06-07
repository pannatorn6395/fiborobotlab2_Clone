#!/usr/bin/env  /usr/bin/python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu
from geometry_msgs.msg import Quaternion
from ament_index_python.packages import get_package_share_directory

import numpy as np
import sys
import RTIMU
import os.path
import time
import math

### IMU setting ###
SETTINGS_FILE = "RTIMULib"
print("Using settings file " + SETTINGS_FILE + ".ini")
if not os.path.exists(SETTINGS_FILE + ".ini"):
    print("Settings file does not exist, will be created")
s = RTIMU.Settings(SETTINGS_FILE)
imu = RTIMU.RTIMU(s)
print("IMU Name: " + imu.IMUName())
if (not imu.IMUInit()):
    print("IMU Init Failed")
    sys.exit(1)
else:
    print("IMU Init Succeeded")
# this is a good time to set any fusion parameters

imu.setSlerpPower(0.02)
imu.setGyroEnable(True)
imu.setAccelEnable(True)
imu.setCompassEnable(True)

poll_interval = imu.IMUGetPollInterval()
print("Recommended Poll Interval: %dmS\n" % poll_interval)


def get_quaternion_from_euler(roll, pitch, yaw):
    qx = np.sin(roll / 2) * np.cos(pitch / 2) * np.cos(yaw / 2) - np.cos(roll / 2) * np.sin(pitch / 2) * np.sin(yaw / 2)
    qy = np.cos(roll / 2) * np.sin(pitch / 2) * np.cos(yaw / 2) + np.sin(roll / 2) * np.cos(pitch / 2) * np.sin(yaw / 2)
    qz = np.cos(roll / 2) * np.cos(pitch / 2) * np.sin(yaw / 2) - np.sin(roll / 2) * np.sin(pitch / 2) * np.cos(yaw / 2)
    qw = np.cos(roll / 2) * np.cos(pitch / 2) * np.cos(yaw / 2) + np.sin(roll / 2) * np.sin(pitch / 2) * np.sin(yaw / 2)

    return [qx, qy, qz, qw]


class Imupublisher(Node):
    def __init__(self):
        global SETTINGS_FILE
        super().__init__('imu_pub')  # Node name
        self.declare_parameters(
            namespace='',
            parameters=[("id", "1")]
        )
        self.pub = self.create_publisher(Imu, "/robot{}/imu".format(self.get_parameter("id").value), 10)
        timer_period = 1 / 100
        self.timer = self.create_timer(timer_period, self.timer_callback)
        self.avaliable = False

    def timer_callback(self):
        global imu
        msg = Imu()
        self.avaliable = imu.IMURead()
        if self.avaliable:
            data = imu.getIMUData()  # {'timestamp', 'fusionPoseValid', 'fusionPose', 'fusionQPoseValid', 'fusionQPose', 'gyroValid', 'gyro', 'accelValid', 'accel', 'compassValid', 'compass', 'pressureValid', 'pressure', 'temperatureValid', 'temperature', 'humidityValid', 'humidity'}
            q = data["fusionQPose"]
            # q = get_quaternion_from_euler(fusionpose[0] , fusionpose[1] , fusionpose[2])
            msg.orientation.x = q[0]
            msg.orientation.y = q[1]
            msg.orientation.z = q[2]
            msg.orientation.w = q[3]
            self.pub.publish(msg)


def main():
    rclpy.init()
    imu_publisher = Imupublisher()
    rclpy.spin(imu_publisher)
    imu_publisher.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
