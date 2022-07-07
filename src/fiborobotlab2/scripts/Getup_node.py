#!/usr/bin/env /usr/bin/python3

import rclpy
from rclpy.node import Node
from fiborobotlab2.msg import Rstate

import time
robotname="robot1"
stainding_tipic='standing_status_{}'.format(robotname)

class Getup(Node):
    def __init__(self):
        super().__init__('Getup')
        self.publisher_ = self.create_publisher(Rstate, stainding_tipic, 10)     # CHANGE
        timer_period = 1 # seconds
        self.timer = self.create_timer(timer_period, self.read_standing_status)
        self.Standing_status=None

    def read_standing_status(self):
        responsePacket_length = 7
        package = [255,255,1,4,2,3,1,244]
        self.serialDevice.write(package)
        time.sleep(0.01)
        responsePacket = self.serialDevice.read(self.serialDevice.inWaiting())
        #print("responsePacket status=", responsePacket)
        #print("responsePacket length=", len(responsePacket))
        if(len(responsePacket) == responsePacket_length):
            # if(responsePacket[5] == 0):
            #     print("robot standing")
            # else:
            #     print("robot falling " + str(responsePacket[5]))
            ##### return to server #####
            self.Standing_status = responsePacket[5]
        else:
            self.Standing_status = None