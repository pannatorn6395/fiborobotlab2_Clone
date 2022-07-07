#!/usr/bin/env  /usr/bin/python3
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import TimerAction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import ComposableNodeContainer
from launch_ros.descriptions import ComposableNode
import select
import os
import numpy as np
import subprocess
from ament_index_python.packages import get_package_share_directory

import simplejson
config_robot = simplejson.load(open(os.path.join(get_package_share_directory('fiborobotlab2'), 'config', 'robotname.json')))
robotname=config_robot["robotname"]
def generate_launch_description():
    return LaunchDescription([
        Node(package='dynamixel_sdk_examples',executable='read_write_node',name='read_write_node_{}'.format(robotname)),
        Node(package='fiborobotlab2',executable='detection_node.py',name='Detection_Node_{}'.format(robotname)),
        Node(package='fiborobotlab2',executable='PID_Node.py',name='PID_node_{}'.format(robotname)),
        Node(package='fiborobotlab2',executable='State_Controll.py',name='State_Controll_{}'.format(robotname)),
        Node(package='fiborobotlab2',executable='Follow_ball.py',name='Fllow_ball_{}'.format(robotname)),
        Node(package='fiborobotlab2',executable='camera.py',name='camera_publisher_{}'.format(robotname)),
        Node(package='fiborobotlab2',executable='prepare_kick.py',name='Prepare_Kick_{}'.format(robotname)),
        Node(package='fiborobotlab2',executable='Kick.py',name='Kick_{}'.format(robotname)),

    ])
