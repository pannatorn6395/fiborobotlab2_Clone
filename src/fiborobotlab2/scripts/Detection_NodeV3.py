#!/usr/bin/env /usr/bin/python3

from pycoral.adapters import common
from pycoral.adapters import detect as detect1
from pycoral.utils.dataset import read_label_file
from pycoral.utils.edgetpu import make_interpreter

import cv2
import numpy as np
import imutils

import time
import detect

from sensor_msgs.msg import Image, CameraInfo
from cv_bridge import CvBridge
from ament_index_python.packages import get_package_share_directory
import tflite_runtime.interpreter as tflite
from pycoral.utils.dataset import read_label_file
from PIL import ImageDraw
from PIL import Image as imag1
import v4l2capture
import os
import select
import rclpy
from rclpy.node import Node
from fiborobotlab2.msg import Rstate
import simplejson
from pycoral.utils.edgetpu import make_interpreter
width, height = 1280, 720
config_robot = simplejson.load(open(os.path.join(get_package_share_directory('fiborobotlab2'), 'config', 'robotname.json')))
robotname=config_robot["robotname"]
Ball_position_topic='Ball_position_{}'.format(robotname)
Robot_state_topic='Robot_state_{}'.format(robotname)
Follow_ball_state_topic='Follow_ball_state_{}'.format(robotname)
camera0_topic='/camera0/image_{}'.format(robotname)
detection_state_topic = 'detection_state_{}'.format(robotname)
test_topic = 'test_{}'.format(robotname)
stainding_tipic='standing_status_{}'.format(robotname)
kick_topic='kick_status_{}'.format(robotname)

MODEL = '/home/fiborobotlab/dev_ws/src/fiborobotlab2/models/efficientdet-lite-ball_goal_edgetpu.tflite'
LABEL = '/home/fiborobotlab/dev_ws/src/fiborobotlab2/models/ball_goal.txt'
WINDOW_NAME = "Preview"
SCAN_BBOX = True
THRESHOLD = 0.4
COUNT = 1 # Number of times to run inference

class Detection_Node(Node):
  def __init__(self):
    super().__init__('Detection_Node')

    self.publisher_ = self.create_publisher(Rstate, Ball_position_topic, 10)     # CHANGE
    self.publisher_2 = self.create_publisher(Rstate, Robot_state_topic, 10)     # CHANGE
    self.publisher_3 = self.create_publisher(Rstate, Follow_ball_state_topic, 10)     # CHANGE
    self.camera0_sub = self.create_subscription(Image, camera0_topic, self.detection, 2)  
    self.subscription = self.create_subscription(Rstate,detection_state_topic,self.Detection_State,10)
    self.publisher_4 = self.create_publisher(Image,test_topic , 10)     # CHANGE
    self.standing_sub = self.create_subscription(Rstate,stainding_tipic,self.standind_status,10)
    self.kick_status_sub = self.create_subscription(Rstate,kick_topic,self.kick_status,10)

    
    self.msg = Rstate()  
    self.labels = read_label_file(LABEL)
    self.interpreter = make_interpreter(MODEL)
    self.interpreter.allocate_tensors()
    self.camera0_sub
    self.standing_sub
    self.kick_status_sub
    self.bridge = CvBridge()
    self.count_scanball=0
    self.count_follow_ball=0
    self.confirmed_time=True
    self.state=None
    self.start_time=time.time()
    self.time_confirmed_detect=time.time()
    self.detection_state="Detect_ball"
    self.Standing_status=None
    self.kick_state=None
  def getup(self):
    self.msg.robot_state="Getup"
    self.publisher_.publish(self.msg)
    self.get_logger().info('robot state: %s ' % (self.msg.robot_state))

  def standind_status(self,msg):
      self.Standing_status = msg.standing_status
  def kick_status(self,msg):
      self.kick_state=msg.kick_status
  def Detection_State(self,msg):
      self.detection_state = msg.detection_state

  def draw_objects(self,draw, objs, labels):
    """Draws the bounding box and label for each object."""
    for obj in objs:
        bbox = obj.bbox
        draw.rectangle([(bbox.xmin, bbox.ymin), (bbox.xmax, bbox.ymax)],
                    outline='blue')
        draw.text((bbox.xmin + 10, bbox.ymin + 10),
                '%s\n%.2f' % (labels.get(obj.id, obj.id), obj.score),
                fill='blue')
  def scan(self,image, interpreter, scan_list, scan_size):
        obj_list = []
        for box in scan_list:
            crop_image = image.crop((box[0], box[1], box[0]+scan_size, box[1]+scan_size))
            _, scale = common.set_resized_input(interpreter, (scan_size, scan_size), lambda size: crop_image.resize(size, imag1.ANTIALIAS))
            # scale = detect.set_input(interpreter, (scan_size, scan_size), lambda size: crop_image.resize(size, Image.ANTIALIAS))    # old edgetpu API
            interpreter.invoke()
            objs = detect1.get_objects(interpreter, THRESHOLD, scale)
            # objs = detect.get_output(interpreter, THRESHOLD, scale)    # old edgetpu API
            for obj in objs:
                bbox = obj.bbox.translate(box[0], box[1])
                obj_list.append(detect1.Object(obj.id, obj.score, bbox))
        return obj_list
  def detection(self,image):
        image=self.bridge.imgmsg_to_cv2(image, desired_encoding='passthrough')
        #cv2.imshow("S",image)
        #cv2.waitKey(1)
        image = imag1.fromarray(image)
        if image.size != (width, height):
            image = image.resize((width, height))

        scan_list = [(0, 0), (width-height, 0)] # Overview left and right
        scan_size = height
        obj_list = self.scan(image, self.interpreter, scan_list, scan_size)
        self.draw_objects(ImageDraw.Draw(image), obj_list, self.labels)
        if (self.Standing_status == 1 or self.Standing_status == 2):
            self.detection_state = "Detect_ball"
            self.msg.robot_state="stop_scanball"
            self.publisher_.publish(self.msg) 
            self.getup()
        else:
            if self.detection_state == "Detect_ball" and self.Standing_status != 1 and self.Standing_status != 2:
                if not obj_list:
                    if time.time() - self.start_time >3:
                        self.state="scanball"
                        self.msg.center_x=int(-1)
                        self.msg.center_y=int(-1)
                        self.msg.robot_state="scanball"
                        self.publisher_.publish(self.msg)
                        self.get_logger().info('Publishing center x: %d center y: %d ' % (self.msg.center_x,self.msg.center_y))
                        self.start_time=time.time()
                        self.count_scanball+=1
                        time_confirmed_detect=time.time()
                        self.msg.follow_ball_status="OFF"
                        self.publisher_3.publish(self.msg)
                else:
                    for obj in obj_list:
                        if obj.id == 0 and obj.score > 0.55:
                            center_x = (obj_list[0].bbox.xmax + obj_list[0].bbox.xmin)/2
                            center_y = (obj_list[0].bbox.ymax + obj_list[0].bbox.ymin)/2
                            self.msg.center_x=int(center_x)
                            self.msg.center_y=int(center_y)
                            if time.time() - self.time_confirmed_detect >4:
                                self.msg.follow_ball_status="Working"
                                self.publisher_3.publish(self.msg)
                                self.time_confirmed_detect=time.time()
                            if self.kick_state != "kick":
                                self.msg.robot_state="followBall"
                                # CHANGE
                            if self.kick_state =="kick":
                                self.msg.robot_state="kickBall"
                            self.publisher_.publish(self.msg)
                            print(self.kick_state)
                            self.get_logger().info('Publishing center x: %d center y: %d ' % (self.msg.center_x,self.msg.center_y))
                        # print(labels.get(obj.id, obj.id))
                        #print('  id:    ', obj.id)
                        #print('  score: ', obj.score)
                        #print('  bbox:  ', obj.bbox)
                            self.start_time=time.time()
                            self.count_scanball=0
                        else:
                            if time.time() - self.start_time >3:
                                self.state="scanball"
                                self.msg.center_x=int(-1)
                                self.msg.center_y=int(-1)
                                self.msg.robot_state="scanball"
                                self.publisher_.publish(self.msg)
                                self.get_logger().info('Publishing center x: %d center y: %d ' % (self.msg.center_x,self.msg.center_y))
                                self.start_time=time.time()
                                self.count_scanball+=1
                                time_confirmed_detect=time.time()
                                self.msg.follow_ball_status="OFF"
                                self.publisher_3.publish(self.msg)

                #print(self.state)
            #print('No objects detected')
            # print(type(image))
            elif self.detection_state == "detect_goal"and self.Standing_status != 1 and self.Standing_status != 2:
                print(22222)
                if not obj_list:
                    if time.time() - self.start_time >3:
                        self.state="scan_goal"
                        self.msg.center_x=int(-1)
                        self.msg.center_y=int(-1)
                        self.msg.robot_state="scan_goal"
                        self.publisher_.publish(self.msg)
                        self.get_logger().info('Publishing center x: %d center y: %d ' % (self.msg.center_x,self.msg.center_y))
                        self.start_time=time.time()
                        self.count_scanball+=1
                        time_confirmed_detect=time.time()
                        self.msg.follow_ball_status="OFF"
                        self.publisher_3.publish(self.msg)
                else:
                    for obj in obj_list:
                        if obj.id == 1 and obj.score > 0.4:
                            center_x = (obj_list[0].bbox.xmax + obj_list[0].bbox.xmin)/2
                            center_y = (obj_list[0].bbox.ymax + obj_list[0].bbox.ymin)/2
                            self.msg.center_x=int(center_x)
                            self.msg.center_y=int(center_y)
                            if time.time() - self.time_confirmed_detect >4:
                                self.msg.follow_ball_status="Working"
                                self.publisher_3.publish(self.msg)
                                self.time_confirmed_detect=time.time()

                            self.msg.robot_state="curve_direction_goal"
                                # CHANGE
                            self.publisher_.publish(self.msg)
                            self.get_logger().info('Publishing center x: %d center y: %d ' % (self.msg.center_x,self.msg.center_y))
                            self.start_time=time.time()
                            self.count_scanball=0
                        else:
                            if time.time() - self.start_time >3:
                                self.state="scan_goal"
                                self.msg.center_x=int(-1)
                                self.msg.center_y=int(-1)
                                self.msg.robot_state="scan_goal"
                                self.publisher_.publish(self.msg)
                                self.get_logger().info('Publishing center x: %d center y: %d ' % (self.msg.center_x,self.msg.center_y))
                                self.start_time=time.time()
                                self.count_scanball+=1
                                time_confirmed_detect=time.time()
                                self.msg.follow_ball_status="OFF"
                                self.publisher_3.publish(self.msg)
            image = cv2.cvtColor(np.array(image), cv2.COLOR_BGR2RGB)
            cv2.imshow("S",image)
            cv2.waitKey(1)
            image_msg = self.bridge.cv2_to_imgmsg(np.array(image), "bgr8")
            image_msg.header.stamp = self.get_clock().now().to_msg()
            self.publisher_4.publish(image_msg)  # Publish image
def main(args=None):
    rclpy.init(args=args)
    robot_state=[0, [0, 0, False, None, 0, 0], [None, None, False], [None, None], [None, 3]]
    Detection = Detection_Node()
    while rclpy.ok():
        rclpy.spin_once(Detection)
    Detection.destroy_node()
    rclpy.shutdown()
if __name__ == '__main__':
    main()
