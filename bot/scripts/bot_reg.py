#!/usr/bin/env python

# This file was written by Tpmonkey (Nuttee)
# to combine features and provide an easy + customizable node setup.

# This file is a part of
# RoboCup@Home Education Workshop & Challenge 2022
#          @ Robocup 2022 Bankok (KU)

# Current supported features
# - Speech: Google TTS, Google RS
# - Vision: Take a photo

from cgitb import text
import rospy
import multiprocessing
# import signal
from std_msgs.msg import *
# from opencv_apps.msg import *

# Speech
import speech_recognition as sr
# from gtts import gTTS

# Vision
# from cv_bridge import CvBridge, CvBridgeError
# import cv2

# import os
text = None

class CustomNode:
    def __init__(self, name, topic = None, msg_type = None):
        self.name = name
        self.pub = None
        
        # Vision
        # self.bridge = CvBridge()

        rospy.init_node(name)
        
        # signal.signal(signal.SIGALRM, self.handler)
        # signal.alarm(10)
        

        if topic:
            # Uncomment to switch between sub and pub.
            self.pub = rospy.Publisher(topic, msg_type, queue_size = 10)       
        
    # def handler(self, signum, frame):
    #     print("Took too long.")
    #     pass
        
    
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
        global text
        while not rospy.is_shutdown():
            text = self.google_rs()
            
            # print("main", self.text)
            
            
            print("main", text)
            
            if text is not None:
                self.pub.publish(text)
        
        # Put this behind.
        rospy.spin()
    
    
    ### Speech
    # google_rs(langauge: str = "en-US") -> Optional[str]
    # speak(text: str) -> None
    def google_rs(self):
        r = sr.Recognizer()
        
        with sr.Microphone() as source:
            print(">>> Say something!")
            # audio = r.listen(source)
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

        
        # print("Recognizing...")
        # p = multiprocessing.Process(target = self.reg, args = [audio, r])
        # # result = r.recognize_google(audio)
        # p.start()
        
        # p.join(6)
        
        # if p.is_alive():
        #     print("Took too long.")
        #     p.terminate()
        

    # def speak(text):
    #     tts = gTTS(text)
        
    #     tts.save("speech.mp3")
    #     os.system("mpg321 -q speech.mp3")
    #     os.remove("speech.mp3")
    
    ### Vision
    # take_photo(data: /topic/rgb/image_raw, image_title: str + ".jpg") -> None
    # def take_photo(self, data, image_title):
    #     try:
    #         cv_image = self.bridge.imgmsg_to_cv2(data), "bgr8"
    #     except CvBridgeError as e:
    #         print(e)

    #     cv2.write(image_title, cv_image)
    #     print("Saved image as", image_title)
    

if __name__ == '__main__':
    csNode = CustomNode(
        name = "bot_reg",
        topic = "bot/voice_input", 
        msg_type = String
    )
    csNode.main()