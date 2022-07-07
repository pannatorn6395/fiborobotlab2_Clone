#!/usr/bin/env  /usr/bin/python3
import rclpy
from rclpy.node import Node
from dynamixel_sdk_custom_interfaces.msg import   SetPosition
from dynamixel_sdk_custom_interfaces.srv import GetPosition
from fiborobotlab2.msg import Rstate
import serial
import time
import copy
import os
from ament_index_python.packages import get_package_share_directory
import threading

import simplejson
config_robot = simplejson.load(open(os.path.join(get_package_share_directory('fiborobotlab2'), 'config', 'robotname.json')))
robotname=config_robot["robotname"]
SetPosition_topic='set_position_{}'.format(robotname)
action_state_topic='action_state_{}'.format(robotname)
stainding_tipic='standing_status_{}'.format(robotname)
class State_Controll(Node):
    def __init__(self):
        super().__init__('State_Controll_{}'.format(robotname))
        self.subscription = self.create_subscription(Rstate,action_state_topic,self.set_locomotion,10)
        self.publisher_ = self.create_publisher(Rstate, stainding_tipic, 10)     # CHANGE
        self.str_comport = '/dev/ttyACM0'
        self.baudrate = 115200
        self.robot_state = None
        self.Standing_status=None
        timer_period = 1 # seconds
        self.timer = self.create_timer(timer_period, self.read_standing_status)
        print("$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$")
        print("Initial Locomotion : comport = " + self.str_comport + " , baudrate = " + str(self.baudrate))
        try:
            self.connect()
            print("Locomotion initial successfully + + +")
        except Exception as e:
            print(e)
            print("Locomotion initial fail!!")

    def set_locomotion(self, msg):
        locomotion_command=msg.action_state
        if(locomotion_command == "stop"):
            self.stop_walk()
        elif(locomotion_command == "forward_berserk"):
            self.forward_walk_berserk()
        elif(locomotion_command == "forward_range"):
            self.forward_walk_range()
        elif(locomotion_command == "forward_almost_normal"):
            self.forward_walk_than_normal()
        elif(locomotion_command == "forward_normal"):
            self.forward_walk_normal()
        ####################################
        elif(locomotion_command == "turn_left"):
            self.turn_left()
        elif(locomotion_command == "turn_right"):
            self.turn_right()
        elif(locomotion_command == "left_kick"):
            self.left_kick()
        elif(locomotion_command == "right_kick"):
            self.right_kick()
        elif(locomotion_command == "getup"):
            self.getup()
        elif(locomotion_command == "left_curve"):
            self.left_curve_walk()   
        elif(locomotion_command == "right_curve"):
            self.right_curve_walk()     
        elif(locomotion_command == "left_save"):
            self.left_save()    
        elif(locomotion_command == "right_save"):
            self.right_save()  

        time.sleep(0.01)
        responsePacket = self.serialDevice.read(self.serialDevice.inWaiting())
    def connect(self):
        self.serialDevice = serial.Serial(port = self.str_comport, baudrate = self.baudrate, timeout=0)
    def open_run_locomotion_process(self):
        self.locomotion_process = threading.Thread(target=self.run_locomotion_process,args=())

        self.locomotion_process.start()
    def close_run_locomotion_process(self):
        self.locomotion_process.joint()
    def run_locomotion_process(self):
        while True:
            self.read_standing_status()
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
            print(self.Standing_status)
            msg_standing=Rstate()
            msg_standing.standing_status=self.Standing_status
            self.publisher_.publish(msg_standing)
        else:
            self.Standing_status = None
    def sit(self):
        package = [255,255,1,4,3,2,51,194]
        self.serialDevice.write(package)
    
    def stand(self):
        package = [255,255,1,4,3,2,52,193]
        self.serialDevice.write(package)

    def getup(self):
        package = [255,255,1,4,3,2,104,141]
        self.serialDevice.write(package)

    def left_kick(self):
        package = [255,255,1,4,3,2,106,139]
        self.serialDevice.write(package)
    
    def right_kick(self):
        package = [255,255,1,4,3,2,107,138]
        self.serialDevice.write(package)

    def left_save(self):
        package = [255,255,1,4,3,2,103,142]
        self.serialDevice.write(package)

    def right_save(self):
        package = [255,255,1,4,3,2,102,143]
        self.serialDevice.write(package)

    def forward_walk_berserk(self, step_flag = False):
        package = [255,255,1,6,3,5,183,127,127,59]
        self.serialDevice.write(package)
        time.sleep(0.1)
        if step_flag == True:
            package = [255,255,1,4,3,2,112,133]
        else:
            package = [255,255,1,4,3,2,111,134]
        self.serialDevice.write(package)

    def forward_walk_range(self, step_flag = False):
        package = [255,255,1,6,3,5,173,127,127,69]
        self.serialDevice.write(package)
        time.sleep(0.1)
        if step_flag == True:
            package = [255,255,1,4,3,2,112,133]
        else:
            package = [255,255,1,4,3,2,111,134]
        self.serialDevice.write(package)
    def forward_walk_than_normal(self, step_flag = False):
        package = [255,255,1,6,3,5,163,127,127,79]
        self.serialDevice.write(package)
        time.sleep(0.1)
        if step_flag == True:
            package = [255,255,1,4,3,2,112,133]
        else:
            package = [255,255,1,4,3,2,111,134]
        self.serialDevice.write(package)
    def forward_walk_normal(self, step_flag = False):
        package = [255,255,1,6,3,5,157,127,127,85]
        self.serialDevice.write(package)
        time.sleep(0.1)
        if step_flag == True:
            package = [255,255,1,4,3,2,112,133]
        else:
            package = [255,255,1,4,3,2,111,134]
        self.serialDevice.write(package)
    def forward_walk_soft(self, step_flag = False):
        package = [255,255,1,6,3,5,173,127,127,69]
        self.serialDevice.write(package)
        time.sleep(0.1)
        if step_flag == True:
            package = [255,255,1,4,3,2,112,133]
        else:
            package = [255,255,1,4,3,2,111,134]
        self.serialDevice.write(package)
    def left_walk(self, step_flag = False):
        package = [255,255,1,6,3,5,127,157,127,85]
        self.serialDevice.write(package)
        time.sleep(0.1)
        if step_flag == True:
            package = [255,255,1,4,3,2,112,133]
        else:
            package = [255,255,1,4,3,2,111,134]
        self.serialDevice.write(package)
    
    def left_curve_walk(self, step_flag = False):
        package = [255,255,1,6,3,5,127,157,112,100]
        self.serialDevice.write(package)
        time.sleep(0.1)
        if step_flag == True:
            package = [255,255,1,4,3,2,112,133]
        else:
            package = [255,255,1,4,3,2,111,134]
        self.serialDevice.write(package)
    
    def backward_walk(self, step_flag = False):
        package = [255,255,1,6,3,5,87,127,127,155]
        self.serialDevice.write(package)
        time.sleep(0.1)
        if step_flag == True:
            package = [255,255,1,4,3,2,112,133]
        else:
            package = [255,255,1,4,3,2,111,134]
        self.serialDevice.write(package)
    
    def right_walk(self, step_flag = False):
        package = [255,255,1,6,3,5,127,97,127,145]
        self.serialDevice.write(package)
        time.sleep(0.1)
        if step_flag == True:
            package = [255,255,1,4,3,2,112,133]
        else:
            package = [255,255,1,4,3,2,111,134]
        self.serialDevice.write(package)

    def right_curve_walk(self, step_flag = False):
        package = [255,255,1,6,3,5,127,92,142,135]
        self.serialDevice.write(package)
        time.sleep(0.1)
        if step_flag == True:
            package = [255,255,1,4,3,2,112,133]
        else:
            package = [255,255,1,4,3,2,111,134]
        self.serialDevice.write(package)
    
    def turn_left(self, step_flag = False):
        package = [255,255,1,6,3,5,127,127,147,95]
        self.serialDevice.write(package)
        time.sleep(0.1)
        if step_flag == True:
            package = [255,255,1,4,3,2,112,133]
        else:
            package = [255,255,1,4,3,2,111,134]
        self.serialDevice.write(package)
    
    def turn_right(self, step_flag = False):
        package = [255,255,1,6,3,5,127,127,107,135]
        self.serialDevice.write(package)
        time.sleep(0.1)
        if step_flag == True:
            package = [255,255,1,4,3,2,112,133]
        else:
            package = [255,255,1,4,3,2,111,134]
        self.serialDevice.write(package)
    
    def stop_walk(self):
        package = [255,255,1,4,3,2,110,135]
        self.serialDevice.write(package)
def main(args=None):
    rclpy.init(args=args)
    Robotstate = State_Controll()
    #rclpy.spin(Detection)
    while rclpy.ok():
        rclpy.spin_once(Robotstate)
    Robotstate.destroy_node()
    rclpy.shutdown()
if __name__ == '__main__':
    main()

