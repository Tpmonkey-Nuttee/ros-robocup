#!/usr/bin/env python
import rospy
from std_msgs.msg import String
import speech_recognition as sr

RESPONDS = {
    "hello": "hello, good morning! how are you!",
    "hi": "fuck you"
}

def googlesr():
    rospy.init_node('regonizer', anonymous=True)
    pub = rospy.Publisher('lm_data', String, queue_size=10)

    while not rospy.is_shutdown():
        # obtain audio from the microphone
        r = sr.Recognizer()
        
        with sr.Microphone() as source:
            print(">>> Say something!")
            #audio = r.listen(source)
            audio = r.record(source, duration=5)
            
        # recognize speech using Google Speech Recognition
        result = None
        try:
            result = r.recognize_google(audio).lower()
            print("SR result: " + result)
        except sr.UnknownValueError:
            print("SR could not understand audio")
        except sr.RequestError as e:
            print("Could not request results from Google Speech Recognition service; {0}".format(e))
        
        if result is not None:
            if result in RESPONDS:
                pub.publish(RESPONDS[result])
            if result == "help":
                pub.publish("take cam")


if __name__ == '__main__':
    try:
        googlesr()
    except rospy.ROSInterruptException:
        pass