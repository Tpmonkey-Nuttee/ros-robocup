#!/usr/bin/env python

# This file was written by Tpmonkey (Nuttee)
# to combine features and provide an easy + customizable node setup.

# This file is a part of
# RoboCup@Home Education Workshop & Challenge 2022
#          @ Robocup 2022 Bankok (KU)

# Current supported features
# - Speech: Google TTS, Google RS
# - Vision: Take a photo

# import queue
import rospy
from std_msgs.msg import *
from opencv_apps.msg import *

# Speech
# import speech_recognition as sr
# from gtts import gTTS

# Vision
# from cv_bridge import CvBridge, CvBridgeError
# import cv2

# import os

### Nodes
# reg
# output topic: bot/voice_input

# speak
# input topic: bot/speak

# camera
# input topic: bot/take_photo

class CustomNode:
    def __init__(self):
        rospy.init_node("bot_main")
        
        self.reg_sub = rospy.Subscriber("bot/voice_input", String, self.heard)
        self.speak_pub = rospy.Publisher("bot/speak", String, queue_size = 10)
        
        self.nav_sub = rospy.Subscriber("bot/reached", String, self.reached)
        self.nav_pub = rospy.Publisher("bot/nav", String, queue_size = 10)

    
    def reached(self, data):
        print("Reached", data.data)
    
    def goto(self, location):
        self.nav_pub.publish(location)
    
    def heard(self, data):
        msg = data.data       
        
        # TODO: 
        self.speak(msg)        
    
    def speak(self, text):
        self.speak_pub.publish(text)

    def callback(self, data):
        print("INFO: ", data)

    

if __name__ == '__main__':
    csNode = CustomNode()
    
    rospy.spin()