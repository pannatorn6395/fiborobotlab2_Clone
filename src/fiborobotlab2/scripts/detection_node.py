#!/usr/bin/env  /usr/bin/python3
import sys
import signal
import argparse
import time
import threading
from sensor_msgs.msg import Image, CameraInfo
from cv_bridge import CvBridge
from ament_index_python.packages import get_package_share_directory
import detect
import tflite_runtime.interpreter as tflite
import platform
import subprocess
from PIL import ImageDraw
from PIL import Image as imag1

import cv2
import numpy as np
import imutils
import v4l2capture
import os
import select
import rclpy
from rclpy.node import Node
from fiborobotlab2.msg import Rstate
import simplejson
MODEL = '/home/fiborobotlab/dev_ws/src/fiborobotlab2/models/output_tflite_graph_edgetpu.tflite'
LABEL = '/home/fiborobotlab/dev_ws/src/fiborobotlab2/models/label.txt'
THRESHOLD = 0.4
COUNT = 1 # Number of times to run inference
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
    self.labels = self.load_labels(LABEL)
    self.interpreter = self.make_interpreter(MODEL)
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
  def load_labels(self,path, encoding='utf-8'): #Returns: Dictionary mapping indices to labels.
    with open(path, 'r', encoding=encoding) as f:
      lines = f.readlines()
      if not lines:
        return {}
      if lines[0].split(' ', maxsplit=1)[0].isdigit():
        pairs = [line.split(' ', maxsplit=1) for line in lines]
        return {int(index): label.strip() for index, label in pairs}
      else:
        return {index: line.strip() for index, line in enumerate(lines)}

  def make_interpreter(self,model_file):
    model_file, *device = model_file.split('@')
    return tflite.Interpreter(
        model_path=model_file,
        experimental_delegates=[
            tflite.load_delegate('libedgetpu.so.1', {'device': device[0]} if device else {})])

  def draw_objects(self,draw, objs, labels):
    """Draws the bounding box and label for each object."""
    for obj in objs:
      bbox = obj.bbox
      draw.rectangle([(bbox.xmin, bbox.ymin), (bbox.xmax, bbox.ymax)],
                    outline='blue')
      draw.text((bbox.xmin + 10, bbox.ymin + 10), '%s\n%.2f' % (labels.get(obj.id, obj.id), obj.score), fill='blue')

  def player(self,objs):
    print(self.detection_state)
    if (self.Standing_status == 1 or self.Standing_status == 2):
      self.detection_state = "Detect_ball"
      self.getup()
    else:
      if self.detection_state == "Detect_ball" and self.Standing_status != 1 and self.Standing_status != 2:
        if not objs:
          if time.time() - self.start_time >2:
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
          for obj in objs:
            if obj.id == 0 and obj.score > 0.7:
              center_x = (obj.bbox.xmax + obj.bbox.xmin)/2
              center_y = (obj.bbox.ymax + obj.bbox.ymin)/2
              self.msg.center_x=int(center_x)
              self.msg.center_y=int(center_y)
              if time.time() - self.time_confirmed_detect >2:
                self.msg.follow_ball_status="Working"
                self.publisher_3.publish(self.msg)
                self.time_confirmed_detect=time.time()
              if self.kick_state != "kick":
                self.msg.robot_state="followBall"
                  # CHANGE
              if self.kick_state =="kick":
                self.msg.robot_state="kickBall"
              self.publisher_.publish(self.msg)
              #time.sleep(0.02)
              print(self.kick_state)
              self.get_logger().info('Publishing center x: %d center y: %d ' % (self.msg.center_x,self.msg.center_y))
            # print(labels.get(obj.id, obj.id))
            #print('  id:    ', obj.id)
            #print('  score: ', obj.score)
            #print('  bbox:  ', obj.bbox)
              self.start_time=time.time()
              self.count_scanball=0
            else:
              if time.time() - self.start_time >2:
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
        if not objs:
          if time.time() - self.start_time >2:
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
          for obj in objs:
            if obj.id == 1 and obj.score > 0.4:
              center_x = (objs[0].bbox.xmax + objs[0].bbox.xmin)/2
              center_y = (objs[0].bbox.ymax + objs[0].bbox.ymin)/2
              self.msg.center_x=int(center_x)
              self.msg.center_y=int(center_y)
              if time.time() - self.time_confirmed_detect >2:
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
              if time.time() - self.start_time >2:
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
  
  def goalkeeper(self, objs):
    print(self.detection_state)
    if (self.Standing_status == 1 or self.Standing_status == 2):
      self.detection_state = "Detect_ball"
      self.getup()
    else:
      if self.detection_state == "Detect_ball" and self.Standing_status != 1 and self.Standing_status != 2:
        if not objs:
          if time.time() - self.start_time >2:
            # self.state="scanball"
            # self.msg.center_x=int(-1)
            # self.msg.center_y=int(-1)
            # self.msg.robot_state="scanball"
            # self.publisher_.publish(self.msg)
            # self.get_logger().info('Publishing center x: %d center y: %d ' % (self.msg.center_x,self.msg.center_y))
            # self.start_time=time.time()
            # self.count_scanball+=1
            # time_confirmed_detect=time.time()
            # self.msg.follow_ball_status="OFF"
            # self.publisher_3.publish(self.msg)
            print("No ball !!")
        else:
          for obj in objs:
            if obj.id == 0 and obj.score > 0.7:
              center_x = (obj.bbox.xmax + obj.bbox.xmin)/2
              center_y = (obj.bbox.ymax + obj.bbox.ymin)/2
              self.msg.center_x=int(center_x)
              self.msg.center_y=int(center_y)
              self.msg.x_line = ((obj.bbox.xmax) - (obj.bbox.xmin)) # Big's CHANGE
              self.msg.y_line = ((obj.bbox.ymax) - (obj.bbox.ymin)) # Big's CHANGE
              if time.time() - self.time_confirmed_detect >2:
                self.msg.follow_ball_status="Working"
                self.publisher_3.publish(self.msg)
                self.time_confirmed_detect=time.time()
              if self.kick_state != "kick":
                self.msg.robot_state="followBall"
              if self.kick_state =="kick":
                self.msg.robot_state="kickBall"
              self.publisher_.publish(self.msg)   # Send
              print(self.kick_state)
              self.get_logger().info('Publishing center x: %d center y: %d ' % (self.msg.center_x,self.msg.center_y))
              self.get_logger().info('Publishing x_line: %d y_line: %d ' % (self.msg.x_line, self.msg.y_line))    # Big's CHANGE
              self.start_time=time.time()
              self.count_scanball=0
            else:
              if time.time() - self.start_time >2:
                # self.state="scanball"
                # self.msg.center_x=int(-1)
                # self.msg.center_y=int(-1)
                # self.msg.robot_state="scanball"
                # self.publisher_.publish(self.msg)
                # self.get_logger().info('Publishing center x: %d center y: %d ' % (self.msg.center_x,self.msg.center_y))
                # self.start_time=time.time()
                # self.count_scanball+=1
                # time_confirmed_detect=time.time()
                # self.msg.follow_ball_status="OFF"
                # self.publisher_3.publish(self.msg)
                print(" Scan ball state ")


  def detection(self,image):
    image=self.bridge.imgmsg_to_cv2(image, desired_encoding='passthrough')
    image = imag1.fromarray(image)
    if image.size != (width, height):
        image = image.resize((width, height))
    scale = detect.set_input(self.interpreter, (width, height), lambda size: image.resize(size, imag1.ANTIALIAS))
    inference_time = 0
    for _ in range(COUNT):
        start = time.perf_counter()
        self.interpreter.invoke()
        inference_time += time.perf_counter() - start
        objs = detect.get_output(self.interpreter, THRESHOLD, scale)
        # print('%.2f ms' % (inference_time * 1000))
        self.draw_objects(ImageDraw.Draw(image), objs, self.labels)
            #####- falling -#####
    if robotname != 'robot4':
      self.player(objs)
    else:
      self.goalkeeper(objs)

    # image = cv2.cvtColor(np.array(image), cv2.COLOR_BGR2RGB)
    # cv2.imshow("S",image)
    # cv2.waitKey(1)
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
