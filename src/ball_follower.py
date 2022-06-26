
from geometry_msgs.msg import Twist
from rclpy.node import Node
from rclpy.qos import QoSProfile
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import LaserScan

import cv2
import numpy as np
from numpy import asarray
import sys
from ctypes import *


class BallFollow(Node):

    def __init__(self):
        super().__init__('ball_follower')

        self.linear_velocity = [0.2, -0.2, 0.0, 0.0, 0.0]  # unit: m/s
        self.angular_velocity = [0.0, 0.5, 0.15, -0.15, 0.0]  # unit: m/s

        libball = cdll.LoadLibrary("./lib/libball_tracking.so")
        self.c_ubyte_p = POINTER(c_ubyte)

        self.ball_tracker = libball.ball_tracking
        self.ball_tracker.restype = POINTER(c_int)

        self.cap = cv2.VideoCapture("qtiqmmfsrc name=qmmf ! video/x-raw, format=NV12, width=640, height=480, framerate=30/1 ! videoconvert ! video/x-raw,format=BGR ! appsink drop=1", cv2.CAP_GSTREAMER)

        qos = QoSProfile(depth=10)
        self.cmd_vel_pub = self.create_publisher(Twist, 'cmd_vel', qos)
        self.cmd_vel_raw_sub = self.create_subscription(Twist, 'cmd_vel_raw', self.cmd_vel_raw_callback, qos)
        self.update_timer = self.create_timer(0.010, self.update_callback)
        self.get_logger().info("Ball Follower Node is Initialized!.")


    def cmd_vel_raw_callback(self, msg):
        self.linear_velocity = msg.linear.x
        self.angular_velocity = msg.angular.z

    def update_callback(self):
        self.detect_ball()

    def detect_ball(self):
        twist = Twist()

        if(self.cap.isOpened()):
            ret, frame = self.cap.read()
            gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
            height, width = gray.shape
            data = asarray(gray)
            data_p = data.ctypes.data_as(self.c_ubyte_p)

            result = self.ball_tracker(height, width, data_p)
            if result[0] is 0:
                twist.linear.x = self.linear_velocity[4]
                twist.angular.z = self.angular_velocity[4]
                self.cmd_vel_pub.publish(twist)
                print('No Result!')
                return 0
            print('Found Circle')
            
            indx = 4
            
            for i in range(1):
                scale_x = int((result[0+i*3]/640) * width)
                scale_y = int((result[1+i*3]/480) * height)
                radius = int((result[2+i*3]/640) * width)
                print(scale_x, radius)
                if(scale_x < 290):
                    indx = 2
                elif(scale_x > 350):
                    indx = 3
                elif(radius > 25):
                    indx = 0

        print(indx, self.linear_velocity[indx], self.angular_velocity[indx])
        twist.linear.x = self.linear_velocity[0]
        twist.angular.z = self.angular_velocity[0]
        self.cmd_vel_pub.publish(twist)
        twist.linear.x = self.linear_velocity[indx]
        twist.angular.z = self.angular_velocity[indx]
        
        self.cmd_vel_pub.publish(twist)
