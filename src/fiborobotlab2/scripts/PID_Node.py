#!/usr/bin/env  /usr/bin/python3
import rclpy
from rclpy.node import Node
from dynamixel_sdk_custom_interfaces.msg import   SetPosition
from dynamixel_sdk_custom_interfaces.srv import GetPosition
from fiborobotlab2.msg import Rstate
from pid_control import PID_Control
import time
import os
from ament_index_python.packages import get_package_share_directory
import threading
scan_path_001 = [[0, 10], [-90, 10], 3] ##[start[pan,tilt], final[pan,tilt], step_amount]
scan_path_002 = [[-90, 45], [90, 45], 6] ##[start[pan,tilt], final[pan,tilt], step_amount]
scan_path_003 = [[90, 10], [0, 10], 3] ##[start[pan,tilt], final[pan,tilt], step_amount]
scan_paths1 = [scan_path_001]
scan_paths2 = [scan_path_002]
scan_paths3 = [scan_path_003]
pan_angle_limit = [-90, 90]
tilt_angle_limit = [-45, 70]
screen_size = [1280, 720]
wait_time_motorMove=0.5
import copy
kp_pan = 0.06
ki_pan = 0
kd_pan = 0.0001

kp_tilt = 0.06
ki_tilt = 0
kd_tilt = 0.0001

dt = 0.01
count_negative=0

import simplejson
config_robot = simplejson.load(open(os.path.join(get_package_share_directory('fiborobotlab2'), 'config', 'robotname.json')))
robotname=config_robot["robotname"]

SetPosition_topic='set_position_{}'.format(robotname)
GetPosition_topic='get_position_{}'.format(robotname)
Ball_position_topic='Ball_position_{}'.format(robotname)
Ball_position_kick_topic='Ball_position_kick_{}'.format(robotname)
action_state_topic='action_state_{}'.format(robotname)
head_state_topic='Head_position_{}'.format(robotname)
goal_position_topic = 'goal_position_{}'.format(robotname)
test_topic = 'test_{}'.format(robotname)
kick_topic='kick_status_{}'.format(robotname)
detection_state_topic = 'detection_state_{}'.format(robotname)

class PID_node(Node):
    def __init__(self):
        super().__init__('PID_node_{}'.format(robotname))
        self.publisher_ = self.create_publisher(SetPosition,SetPosition_topic , 10)     # CHANGE
        self.cli = self.create_client(GetPosition, GetPosition_topic)
        self.subscription = self.create_subscription(Rstate,Ball_position_topic,self.Calculate_PID,10)
        self.subscription2 = self.create_subscription(Rstate,kick_topic,self.Set_head_kick,10)
        self.publisher_2 = self.create_publisher(Rstate, action_state_topic, 10) 
        self.publisher_3 = self.create_publisher(Rstate, head_state_topic, 10) 
        self.publisher_4 = self.create_publisher(Rstate, goal_position_topic, 10) 
        self.publisher_5 = self.create_publisher(Rstate, Ball_position_kick_topic, 10) 

        self.publisher_7 = self.create_publisher(Rstate, detection_state_topic, 10)     # CHANGE
        self.publisher_6 = self.create_publisher(Rstate, kick_topic, 10)     # CHANGE
        # timer_period = 0.5
        # self.timer = self.create_timer(timer_period, self.timer_callback)
        # self.position = 0
        # self.ID = 1
        # self.req = GetPosition.Request()
        # self.count=0
        self.subscription
        self.subscription2
        self.center_x=0
        self.center_y=0
        self.pan = 41
        self.tilt=42
        self.screen_size = screen_size
        self.screen_center_x = self.screen_size[0]/2
        self.screen_center_y = self.screen_size[1]/2
        self.pid_pan = PID_Control(kp_pan, ki_pan, kd_pan,dt)
        self.pid_tilt = PID_Control(kp_tilt, ki_tilt, kd_tilt, dt)
        self.robot_state=None
        self.set_head_kick=None

        self.req = GetPosition.Request()

    def Set_head_kick(self,msg):
        self.set_head_kick=msg.kick_status
    
    def Calculate_PID(self,msg):
        self.get_logger().info('subscript center x: %d center y: %d ' % (msg.center_x,msg.center_y))
        self.center_x=msg.center_x
        self.center_y=msg.center_y
        self.robot_state=msg.robot_state
        #print(self.robot_state)
        # print(self.robot_state)
        # print("5657")
        self.send_request()
        self.send_request2()

        
    def send_request(self): 
        self.req.id = 41   
        self.future = self.cli.call_async(self.req)
    def send_request2(self): 
        self.req.id = 42   
        self.future_2 = self.cli.call_async(self.req)

def convertMotorValueToAngleDeg(motor_value):
    angle_deg = (motor_value * 360 / 4096) - 180
    return angle_deg
def convertAngleDegToMotorValue(angle_deg):
    motor_value = (angle_deg + 180) * 4096 / 360
    return motor_value
def setPosition(motor, value):
        if(value >= -180 and value < 180):
            motor_value = int(convertAngleDegToMotorValue(value))
        else:
            print("Value out of bound")
        return motor_value
    

def main(args=None):
    rclpy.init(args=args)
    PID = PID_node()
    count_pan=0
    count_negative=0
    i=0
    y=10
    a=0
    while rclpy.ok():
        rclpy.spin_once(PID)
        if PID.robot_state=="Getup" :
           print("getup state")
           msg_action=Rstate()
           msg_action.action_state="stop"
           msg = SetPosition()   
           msg.id=PID.pan                                 # CHANGE
           msg.position = int(2048)
           PID.publisher_.publish(msg)
           msg.id=PID.tilt                                 # CHANGE
           msg.position = int(2048)
           PID.publisher_.publish(msg)
           msg_action.action_state="getup"
           PID.publisher_2.publish(msg_action)
           msg_action.action_state="stop"
           PID.publisher_2.publish(msg_action)
           PID.robot_state=None
        elif PID.set_head_kick == 'set_head_kick':
           msg_action=Rstate()
           msg = SetPosition()   
           msg.id=PID.pan                                 # CHANGE
           msg.position = int(convertAngleDegToMotorValue(0))
           PID.publisher_.publish(msg)
           msg.id=PID.tilt                                 # CHANGE
           msg.position = int(convertAngleDegToMotorValue(70))
           PID.publisher_.publish(msg)
           if PID.future.done() and PID.future_2.done():
                try:
                    response = PID.future.result()
                    response2 = PID.future_2.result()
                except Exception as e:
                    PID.get_logger().info(
                        'Service call failed %r' % (e,))
                else:
                    motor_position_x=int(convertMotorValueToAngleDeg(response.position))
                    motor_position_y=int(convertMotorValueToAngleDeg(response2.position))
                    if motor_position_x < 10 and motor_position_y >60:
                        msg_detection_status=Rstate()
                        msg_detection_status.detection_state="Detect_ball"
                        PID.publisher_7.publish(msg_detection_status)
                        msg_kick_status=Rstate()
                        msg_kick_status.kick_status="kick"
                        PID.publisher_6.publish(msg_kick_status)
                        PID.robot_state=None


        else:
            if PID.future.done() and PID.future_2.done():
                try:
                    response = PID.future.result()
                    response2 = PID.future_2.result()
                except Exception as e:
                    PID.get_logger().info(
                        'Service call failed %r' % (e,))
                else:
                    presentposition=response.position
                    # PID.get_logger().info(
                    #     'Present_position : %d' %
                    #     ( response.position))
                    # PID.get_logger().info(
                    #     'Present_position2 : %d' %
                    #     ( response2.position))
                    # print(response.position)
                    # print(response2.position)
                    if PID.center_x >0:
                        i=0
                        y=10
                        a=0
                        count_pan=0   
                        count_negative=0          
                        motor_position_x=convertMotorValueToAngleDeg(response.position)
                        object_position_x=PID.center_x
                        motor_position_y=convertMotorValueToAngleDeg(response2.position)
                        object_position_y=PID.center_y

                        # print(PID.center_x,PID.center_y)
                        motor_position_next_x = motor_position_x + PID.pid_pan.update(PID.screen_center_x, object_position_x)
                        if (motor_position_next_x > pan_angle_limit[1]): motor_position_next_x = pan_angle_limit[1]
                        elif (motor_position_next_x < pan_angle_limit[0]): motor_position_next_x = pan_angle_limit[0]
                        motor_position_next_y = motor_position_y + (-1)*PID.pid_tilt.update(PID.screen_center_y, object_position_y)
                        if (motor_position_next_y > tilt_angle_limit[1]): motor_position_next_y = tilt_angle_limit[1]
                        elif (motor_position_next_y < tilt_angle_limit[0]): motor_position_next_y = tilt_angle_limit[0]
                        # print(motor_position_next_x)
                        # print(motor_position_next_y)
                        head_pan=motor_position_next_x
                        head_tilt=motor_position_next_y
                        motor_position_next_x=convertAngleDegToMotorValue(motor_position_next_x)
                        motor_position_next_y=convertAngleDegToMotorValue(motor_position_next_y)
                        # print(motor_position_next_x)
                        # print(motor_position_next_y)
                        presentx=response.position
                        presenty=response2.position
                        deltax=(int(motor_position_next_x)-int(response.position))/5
                        deltay=(int(motor_position_next_y)-int(response2.position))/5
                        count=0
                        if PID.robot_state=="followBall"  :
                            while(count<5):
                                ### Head Follow Ball ###
                                msg = SetPosition()   
                                msg.id=PID.pan                                 # CHANGE
                                msg.position = int(presentx+deltax)
                                #print(presentx+deltax)
                                PID.publisher_.publish(msg)
                                msg.id=PID.tilt                                 # CHANGE
                                msg.position = int(presenty+deltay)
                                PID.publisher_.publish(msg)
#################################################################################################
                                ### Cheack turn left , turn right or folloball ###
                                msg_head_position=Rstate()
                                msg_head_position.pan_position=int(convertMotorValueToAngleDeg(presentx+deltax))
                                # print("head_pan : "+str(int(convertMotorValueToAngleDeg(presentx+deltax))))
                                msg_head_position.tilt_position=int(convertMotorValueToAngleDeg(presenty+deltay))
                                # print("head_tilt : "+str(int(convertMotorValueToAngleDeg(presenty+deltay))))
                                PID.publisher_3.publish(msg_head_position)
                                presentx=presentx+deltax
                            #print(presenty+deltay)
                                presenty=presenty+deltay
                                count+=1
#################################################################################################
                        elif PID.robot_state=="curve_direction_goal":
                            ### Head Follow Ball ###
                            print(count)
                            while(count<5):
                                msg = SetPosition()   
                                msg.id=PID.pan                                 # CHANGE
                                msg.position = int(presentx+deltax)
                                #print(presentx+deltax)
                                PID.publisher_.publish(msg)
                                msg.id=PID.tilt                                 # CHANGE
                                msg.position = int(presenty+deltay)
                                PID.publisher_.publish(msg)
                                msg_head_position=Rstate()
                                msg_head_position.pan_position=int(convertMotorValueToAngleDeg(presentx+deltax))
                                PID.publisher_4.publish(msg_head_position)
                                presentx=presentx+deltax
                            #print(presenty+deltay)
                                presenty=presenty+deltay
                                count+=1
                        elif PID.robot_state=="kickBall":
                            print(545648)
                            msg_position=Rstate()
                            msg_position.center_x=int(object_position_x)
                            msg_position.center_y=int(object_position_y)
                            PID.publisher_5.publish(msg_position)
                print(PID.robot_state)                
            elif PID.center_x <0:
                count_negative+=1
                if count_negative>2:
                    if PID.robot_state=="scanball":
                        msg=Rstate()
                        msg.action_state="stop"
                        #time.sleep(0.2)
                        PID.publisher_2.publish(msg)
                      
                        print(PID.robot_state)
                        msg = SetPosition()   
                        msg.id=PID.pan
                        msg.position = int(convertAngleDegToMotorValue(i))       
                        PID.publisher_.publish(msg)
                        msg.id=PID.tilt
                        msg.position = int(convertAngleDegToMotorValue(y))       
                        PID.publisher_.publish(msg)            ## wait motors in position ##
                        # time.sleep(0.1)
                        if a==0:
                            i=i+30
                            if i>=90:
                                a=1
                                y=55
                        elif a==1:
                            i=i-30
                            if i<=-90:
                                a=2
                                y=10
                        elif a==2:
                            msg=Rstate()
                            msg.action_state="turn_right"
                            print("turn_right")
                            PID.publisher_2.publish(msg)
                            time.sleep(2)
                            msg.action_state="stop"
                            time.sleep(1)
                            PID.publisher_2.publish(msg)
                            a=0    
                    elif PID.robot_state=="scan_goal":
                        msg=Rstate()
                        msg.action_state="stop"
                        #time.sleep(0.2)
                        PID.publisher_2.publish(msg)
                      
                        print(PID.robot_state)
                        msg = SetPosition()   
                        msg.id=PID.pan
                        msg.position = int(convertAngleDegToMotorValue(i))       
                        PID.publisher_.publish(msg)
                        msg.id=PID.tilt
                        msg.position = int(convertAngleDegToMotorValue(y))       
                        PID.publisher_.publish(msg)            ## wait motors in position ##
                        # time.sleep(0.1)
                        if a==0:
                            i=i+30
                            if i>=90:
                                a=1
                                y=10
                        elif a==1:
                            i=i-30
                            if i<=-90:
                                a=2
                                y=10  
                        elif a==2:
                            msg=Rstate()
                            msg.action_state="right_curve"
                            print("right_curve")
                            PID.publisher_2.publish(msg)
                            time.sleep(2)
                            msg.action_state="stop"
                            time.sleep(1)
                            PID.publisher_2.publish(msg)
                            a=0    
                        # msg=Rstate()
                        # msg.action_state="stop"
                        # PID.publisher_2.publish(msg)
                        # if count_pan==0:                 
                        #     for path in scan_paths1:
                        #         #print(path)
                        #         ## set motors ready position ##
                        #         print(PID.robot_state)
                        #         msg = SetPosition()   
                        #         msg.id=PID.pan
                        #         msg.position = int(convertAngleDegToMotorValue(path[0][0]))       
                        #         PID.publisher_.publish(msg)
                        #         msg.id=PID.tilt
                        #         msg.position = int(convertAngleDegToMotorValue(path[0][1]))       
                        #         PID.publisher_.publish(msg)            ## wait motors in position ##
                        #         time.sleep(0.5)
                        #         count_pan+=1
                                
                        # elif count_pan==1:                 
                        #     for path in scan_paths2:
                        #         #print(path)
                        #         ## set motors ready position ##
                        #         print(PID.robot_state)
                        #         msg = SetPosition()   
                        #         msg.id=PID.pan
                        #         msg.position = int(convertAngleDegToMotorValue(path[0][0]))       
                        #         PID.publisher_.publish(msg)
                        #         msg.id=PID.tilt
                        #         msg.position = int(convertAngleDegToMotorValue(path[0][1]))       
                        #         PID.publisher_.publish(msg)            ## wait motors in position ##
                        #         time.sleep(0.5)
                        #         count_pan+=1
                                
                        # elif count_pan==2:                 
                        #     for path in scan_paths3:
                        #         #print(path)
                        #         ## set motors ready position ##
                        #         print(PID.robot_state)
                        #         msg = SetPosition()   
                        #         msg.id=PID.pan
                        #         msg.position = int(convertAngleDegToMotorValue(path[0][0]))       
                        #         PID.publisher_.publish(msg)
                        #         msg.id=PID.tilt
                        #         msg.position = int(convertAngleDegToMotorValue(path[0][1]))       
                        #         PID.publisher_.publish(msg) 
                        #                     ## wait motors in position ##
                        #         time.sleep(0.5)
                        #         count_pan+=1
                        # elif count_pan==3:
                        #     msg=Rstate()
                        #     if PID.robot_state=="scanball":
                        #         msg.action_state="turn_right"
                        #         print("turn_right")
                        #     elif PID.robot_state=="scan_goal":
                        #         msg.action_state="right_curve"
                        #         print("right_curve")
                        #     PID.publisher_2.publish(msg)
                        #     time.sleep(2)
                        #     msg.action_state="stop"
                        #     time.sleep(1)
                        #     PID.publisher_2.publish(msg)
                        #     count_pan=0             


                        
 




if __name__ == '__main__':
    main()
