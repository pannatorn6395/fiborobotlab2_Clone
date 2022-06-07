#!/home/pannatorn/anaconda3/bin/python
import sys
import signal
import argparse
import time
import threading

from PIL import Image
from PIL import ImageDraw

import detect
import tflite_runtime.interpreter as tflite
import platform
import subprocess

import cv2
import numpy as np
import imutils
import v4l2capture
import os
import select
import rclpy
from rclpy.node import Node
from dynamixel_sdk_custom_interfaces.msg import   SetPosition
from dynamixel_sdk_custom_interfaces.srv import GetPosition
from fiborobotlab2.msg import BBox


#width, height = 1920, 1080
width, height = 1280, 720
WINDOW_NAME = "Preview"
full_screen = False
MODEL = '/home/pannatorn/dev_ws/src/fiborobotlab2/models/output_tflite_graph_edgetpu.tflite'
LABEL = '/home/pannatorn/dev_ws/src/fiborobotlab2/models/label.txt'
THRESHOLD = 0.4
COUNT = 1 # Number of times to run inference
class Detection_Node(Node):
  def __init__(self):
    super().__init__('Detection_Node')
    self.publisher_ = self.create_publisher(BBox, 'Ball_position', 10)     # CHANGE
    self.cli = self.create_client(GetPosition, 'get_position')
    self.req = GetPosition.Request()

    self.send_request()

  def send_request(self):
        self.req.id = 1
        self.future = self.cli.call_async(self.req)
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
                    outline='red')
      draw.text((bbox.xmin + 10, bbox.ymin + 10), '%s\n%.2f' % (labels.get(obj.id, obj.id), obj.score), fill='red')

  def ball_detection(self,robot_state):
    labels = self.load_labels(LABEL)
    interpreter = self.make_interpreter(MODEL)
    interpreter.allocate_tensors()
    cap = v4l2capture.Video_device("/dev/video0")
    size_x, size_y = cap.set_format(width, height, fourcc='MJPG')

    subprocess.call(["v4l2-ctl", "-c", "focus_auto=0"]) ##trun off auto focus##
    subprocess.call(["v4l2-ctl", "-c", "white_balance_temperature_auto=0"]) ##trun off auto white_balance##


    cap.create_buffers(1)
    cap.queue_all_buffers()
    cap.start()
    msg = BBox()                                           # CHANGE
    while True:
      
      select.select((cap,), (), ())
      image_data = cap.read_and_queue()
      img = cv2.imdecode(np.frombuffer(image_data, dtype=np.uint8), cv2.IMREAD_COLOR)
      image = cv2.cvtColor(img, cv2.COLOR_BGR2RGB)
      image = Image.fromarray(image) # image.size = (width, height)
      if image.size != (width, height):
          img = imutils.resize(img, height=height, width=width)
          image = image.resize((width, height))
      scale = detect.set_input(interpreter, (width, height), lambda size: image.resize(size, Image.ANTIALIAS))

      inference_time = 0
      for _ in range(COUNT):
        start = time.perf_counter()
        interpreter.invoke()
        inference_time += time.perf_counter() - start
        objs = detect.get_output(interpreter, THRESHOLD, scale)
        # print('%.2f ms' % (inference_time * 1000))
        self.draw_objects(ImageDraw.Draw(image), objs, labels)

      #print('-------RESULTS--------')
      if not objs:
          robot_state[2][0] = None
          robot_state[2][1] = None
        #print('No objects detected')
      for obj in objs:
          if obj.id == 0 and obj.score > 0.5:
              center_x = (obj.bbox.xmax + obj.bbox.xmin)/2
              center_y = (obj.bbox.ymax + obj.bbox.ymin)/2
              robot_state[2][0] = center_x
              robot_state[2][1] = center_y

              msg.center_x=int(center_x)
              msg.center_y=int(center_y)                      # CHANGE
              self.publisher_.publish(msg)
              self.get_logger().info('Publishing center x: %d center y: %d ' % (msg.center_x,msg.center_y))
              # print(labels.get(obj.id, obj.id))
              #print('  id:    ', obj.id)
              #print('  score: ', obj.score)
              #print('  bbox:  ', obj.bbox)
      cv2.imshow(WINDOW_NAME, img)
      cv2.imshow(WINDOW_NAME, cv2.cvtColor(np.asarray(image), cv2.COLOR_RGB2BGR))
      key = cv2.waitKey(1)
      if key == ord('q'):
        break
      elif key == ord('f'):
        full_screen = not full_screen
        if full_screen: cv2.setWindowProperty(WINDOW_NAME, cv2.WND_PROP_FULLSCREEN, cv2.WINDOW_FULLSCREEN)
        else: cv2.setWindowProperty(WINDOW_NAME, cv2.WND_PROP_FULLSCREEN, cv2.WINDOW_NORMAL)
    cv2.destroyAllWindows()

def main(args=None):
    rclpy.init(args=args)
    robot_state=[0, [0, 0, False, None, 0, 0], [None, None, False], [None, None], [None, 3]]
    Detection = Detection_Node()
    #rclpy.spin(Detection)
    Detection.send_request()
    while rclpy.ok():
        Detection.ball_detection(robot_state)
        rclpy.spin_once(Detection)
    Detection.destroy_node()
    rclpy.shutdown()
if __name__ == '__main__':
    main()