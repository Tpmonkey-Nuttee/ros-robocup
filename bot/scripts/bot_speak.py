#!/usr/bin/env python

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

# Speech
from gtts import gTTS

import os

# Disable warning.
import urllib3
urllib3.disable_warnings(urllib3.exceptions.InsecureRequestWarning)


class CustomNode:
    def __init__(self, name, topic = None, msg_type = None):
        self.name = name    
        rospy.init_node(name)

        rospy.Subscriber(topic, msg_type, self.callback)  
    
    def main(self):        
        # Put this behind.
        rospy.spin()

    def callback(self, data):
        print("INFO: ", data)
        
        self.speak(data.data)

    def speak(self, text):
        print("Saying", text)
        tts = gTTS(text)
        
        tts.save("speech.mp3")
        os.system("mpg321 -q speech.mp3")
        os.remove("speech.mp3")

if __name__ == '__main__':
    csNode = CustomNode(
        name = "bot_speak",
        topic = "bot/speak", 
        msg_type = String
    )
    csNode.main()