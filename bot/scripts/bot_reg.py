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
import speech_recognition as sr

class CustomNode:
    def __init__(self, name, topic = None, msg_type = None):
        self.name = name
        rospy.init_node(name)  
        
        self.pub = rospy.Publisher(topic, msg_type, queue_size = 10)       
    
    def main(self):
        while not rospy.is_shutdown():
            text = self.google_rs()            
            
            if text is not None:
                print("sending", text)
                self.pub.publish(text)
    
    def google_rs(self):
        r = sr.Recognizer()
        
        with sr.Microphone() as source:
            print(">>> Say something!")
            audio = r.record(source, duration=5)
            
        # recognize speech using Google Speech Recognition        
        self.text = None
        try:
            self.text = r.recognize_google(audio)    
            print("SR:", self.text)    
        except sr.UnknownValueError:
            print("SR could not understand audio")
        except sr.RequestError as e:
            print("Could not request results from Google Speech Recognition service; {0}".format(e))        

        return self.text
    

if __name__ == '__main__':
    csNode = CustomNode(
        name = "bot_reg",
        topic = "bot/voice_input", 
        msg_type = String
    )
    csNode.main()