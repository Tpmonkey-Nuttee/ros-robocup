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
from opencv_apps.msg import *
from sensor_msgs.msg import Image

# Speech
# import speech_recognition as sr
# from gtts import gTTS

from subprocess import call
import time

# Vision
from cv_bridge import CvBridge, CvBridgeError
import cv2

import os

class CustomNode:
    def __init__(self, name, topic = None, msg_type = None):
        self.name = name
        self.pub = None
        
        # Vision
        self.bridge = CvBridge()
        self.image = None

        rospy.init_node(name)
        # roslaunch usb_cam usb_cam-test.launch
        # roslaunch rchomeedu_vision multi_astra.launch
        # call("roslaunch usb_cam usb_cam-test.launch")

        if topic:
            # Uncomment to switch between sub and pub.

            rospy.Subscriber(topic, msg_type, self.callback)
            
            # camera/rgb/image_raw
            rospy.Subscriber("usb_cam/image_raw", Image, self.callimage)
            # self.pub = rospy.Publisher(topic, msg_type, queue_size = 10)       
        
    
    def main(self):
        # Put stuff here.
        
        # ---- Note
        ### Speech
        # google_rs(langauge: str = "en-US") -> Optional[str]
        # speak(text: str) -> None
        # 
        ### Vision
        # take_photo(data: /topic/rgb/image_raw, image_title: str + ".jpg") -> None
        #

        
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
    
    ### Speech
    # google_rs(langauge: str = "en-US") -> Optional[str]
    # # speak(text: str) -> None
    # def google_rs(self, langauge = "en-US"):
    #     r = sr.Recognizer()
        
    #     with sr.Microphone() as source:
    #         print(">>> Say something!")
    #         # audio = r.listen(source)
    #         audio = r.record(source, duration=5)
            
    #     # recognize speech using Google Speech Recognition
    #     result = None
    #     try:
    #         print("Recognizing...")
    #         result = r.recognize_google(audio, langauge = langauge)
    #         print("SR result: " + result)
    #     except sr.UnknownValueError:
    #         print("SR could not understand audio")
    #     except sr.RequestError as e:
    #         print("Could not request results from Google Speech Recognition service; {0}".format(e))
        
    #     return result

    # def speak(text):
    #     tts = gTTS(text)
        
    #     tts.save("speech.mp3")
    #     os.system("mpg321 -q speech.mp3")
    #     os.remove("speech.mp3")
    
    ### Vision
    # take_photo(data: /topic/rgb/image_raw, image_title: str + ".jpg") -> None
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