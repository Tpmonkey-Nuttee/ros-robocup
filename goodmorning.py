import rospy
from std_msgs.msg import String
import speech_recognition as sr

RESPONDS = {
    "hello": "hello, good morning! how are you!",
}

def main():
    rospy.init_node("goodmorning")

    pub = rospy.Publisher("lm_data", String, queue_size = 10)

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
            result = r.recognize_google(audio)
            print("SR result: " + result)
        except sr.UnknownValueError:
            print("SR could not understand audio")
        except sr.RequestError as e:
            print("Could not request results from Google Speech Recognition service; {0}".format(e))
        
        if result is not None:
            if result in RESPONDS:
                pub.publish(RESPONDS[result])
            else:
                pub.publish(result)

if __name__ == "__main__":
    try:
        main()
    except rospy.ROSInterruptException:
        pass
