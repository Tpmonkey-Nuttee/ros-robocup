#!/usr/bin/env python
import rospy
from opencv_apps.msg import RotatedRectStamped
from gtts import gTTS
import os

import urllib3
urllib3.disable_warnings(urllib3.exceptions.InsecureRequestWarning)

def saythis(msg):
    print("saying", msg)
    tts = gTTS(msg)
    tts.save("speech.mp3")
    os.system("mpg321 -q speech.mp3")
    os.remove("speech.mp3")

xMax, yMax = 640, 480

def callback(data):
    if rospy.get_time() % 3 > 2.95:
        center = data.rect.center
        print("taking picture.")
        print(center.x, center.y)

        if center.x == 0 and center.y == 0:
            return
        message = []

        if center.x > xMax - 100:
            message.append("go right")
            
        elif center.x < 100:
            message.append("go left")
        
        if center.y < 80:
            message.append("go down")
        elif center.y > yMax - 80:
            message.append("go up")
        
        print(message)
        if message:
            saythis("please " + " and ".join( message))
            rospy.sleep(2)
    
def googletts():
    rospy.init_node('object_middle')
    print("init node")

    rospy.Subscriber("camshift/track_box", RotatedRectStamped, callback)
    print("sub!")

    rospy.spin()

if __name__ == '__main__':
    googletts()