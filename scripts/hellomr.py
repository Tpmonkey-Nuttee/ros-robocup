#!/usr/bin/env python
import rospy
from opencv_apps.msg import FaceArrayStamped
from gtts import gTTS
import os

import urllib3
urllib3.disable_warnings(urllib3.exceptions.InsecureRequestWarning)

class Hello:
    def __init__(self):
        self.faces = []
        self.names = []
        # Take a pic every 4 sec
        self.pic_time = 4

    def saythis(self, msg):
        print("saying", msg)
        tts = gTTS(msg)
        tts.save("speech.mp3")
        os.system("mpg321 speech.mp3")
        os.remove("speech.mp3")
    
    def check(self, faces):
        if len(self.faces) != len(faces):
            return True
        
        return self.faces != faces

    def callback(self, data):

        if rospy.get_time() % self.pic_time > self.pic_time - .05:

            if self.check(data.faces): # Name is diff fronm the last image.
                names = []
                for i in data.faces:
                    if i.label not in self.names:
                        # if not alredy greeted.
                        names.append(i.label)
                        self.names.append(i.label)
                        print("Adding name", i.label)
                
                if names:
                    self.saythis("Hello, " + " and ".join(names))
                
            for i in self.names:
                # if already greeted and left
                if i not in [j.label for j in data.faces ]:
                    print(i, "has left.")
                    self.names.remove(i)
    


    def googletts(self):
        rospy.init_node('greeting')
        print("init node")

        rospy.Subscriber("face_recognition/output", FaceArrayStamped, self.callback)
        print("sub!")

        rospy.spin()

if __name__ == '__main__':
    hello = Hello()
    hello.googletts()