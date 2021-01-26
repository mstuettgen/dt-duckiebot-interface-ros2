#!/usr/bin/env python3

import io
import os
import yaml
import copy
import numpy as np
from threading import Thread
import _thread as thread
from time import sleep

import sys
import rclpy
from rclpy.node import Node

from sensor_msgs.msg import CompressedImage, CameraInfo
from sensor_msgs.srv import SetCameraInfo

import acapture
import cv2
from cv_bridge import CvBridge

class AcaptureCameraNode(Node):

    def __init__(self):
        super().__init__('acapture_camera_node', allow_undeclared_parameters=True, automatically_declare_parameters_from_overrides=True)

        self.node_name = self.get_name()
        self.log = self.get_logger()

        self.cap = acapture.open(0) #/dev/video0

        #self.cap.set(cv2.CAP_PROP_FRAME_WIDTH, 640)
        #self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)
        #self.cap.set(cv2.CAP_PROP_FPS, 5)
        
        self.log.info("successfully opened camera device!")

        # init ROS opencv bridge
        self.bridge = CvBridge()
        self.log.info("sucessfully created CvBridge.")

        
        self.pub_img = self.create_publisher(CompressedImage,"image/compressed",1)
        self.image_msg = CompressedImage()


    def start_capturing(self):

        self.log.info("Start capturing.")

        while True:
            try:
                check,frame = self.cap.read() # non-blocking
                if check:
                    self.log.info("sucessfully read a frame")
                    frame = cv2.cvtColor(frame,cv2.COLOR_BGR2RGB)
                    self.image_msg = self.bridge.cv2_to_compressed_imgmsg(frame)
                    self.image_msg.header.stamp = self.get_clock().now().to_msg()
#                    self.image_msg.frame_id = "camera_optical_frame"
                    self.pub_img.publish(self.image_msg)
                    self.log.info("publishing image")
#                    cv2.waitKey(100)
                    sleep(0.05)

            except StopIteration:
                pass

        self.log.info("Capture Ended.")        


def main(args = None):
    if args is None:
        args = sys.argv

    rclpy.init(args=args)

    node = AcaptureCameraNode()
    thread.start_new_thread(node.start_capturing, ())

 #   rate = node.create_rate(10)
      

    try:

        
#        node.start_capturing()
        rclpy.spin(node)
#        rate.sleep()
    except KeyboardInterrupt:
        pass

    node.destroy_node()
    rclpy.shutdown()
    

if __name__ == '__main__':
    main()
