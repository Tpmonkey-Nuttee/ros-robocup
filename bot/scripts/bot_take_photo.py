#!/usr/bin/env python

# NOTE: We did not use this file in the real demonstration.

# This file was written by Tpmonkey (Nuttee)
# to combine features and provide an easy + customizable node setup.

# This file is a part of
# RoboCup@Home Education Workshop & Challenge 2022
#          @ Robocup 2022 Bankok (KU)

# Current supported features
# - Speech: Google TTS, Google RS
# - Vision: Take a photo

import rospy
from std_msgs.msg import *
from opencv_apps.msg import *
from sensor_msgs.msg import Image

import time

# Vision
from cv_bridge import CvBridge, CvBridgeError
import cv2

import os

class CustomNode:
    def __init__(self, name, topic = None, msg_type = None):
        self.name = name
        rospy.init_node(name)
        
        # Vision
        self.bridge = CvBridge()
        self.image = None

        rospy.Subscriber(topic, msg_type, self.callback)
        
        # camera/rgb/image_raw
        rospy.Subscriber("usb_cam/image_raw", Image, self.callimage)    
        
    
    def main(self):
        # Put stuff here.


        # Put this behind.
        rospy.spin()
    
    def callimage(self, data):
        try:
            self.image = self.bridge.imgmsg_to_cv2(data, "bgr8")
        except CvBridgeError as e:
            print(e)


    def callback(self, data):
        print("INFO: ", data)
        
        title = str(time.time()) + ".jpg"
        self.take_photo(title)
    
    def take_photo(self, image_title):
        cv2.imwrite(image_title, self.image)
        print("Saved image as", image_title)
    

if __name__ == '__main__':
    csNode = CustomNode(
        name = "bot_take_photo",
        topic = "bot/take_photo", 
        msg_type = String
    )
    csNode.main()