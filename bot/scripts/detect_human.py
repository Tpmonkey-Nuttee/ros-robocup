#!/usr/bin/env python

import rospy
from robot_vision_msgs.msg import BoundingBoxes
from std_msgs.msg import String

class Node:
    def __init__(self):

        self.enable = False

        rospy.Subscriber("bot/start_human_dec", String, self.callenable)
        rospy.Subscriber("yolo_ros/bounding_boxes", BoundingBoxes, self.callback)
        
        self.pub = rospy.Publisher("bot/found_human", String, queue_size=10)

    def callback(self, data):
        if not self.enable:
            return

        for bounding_box in data.bounding_boxes:
            if bounding_box.Class == 'person':
                self.enable = False
                self.pub.publish("FoundPerson")
                break

    def callenable(self, data):
        self.enable = True


if __name__ == '__main__':
    rospy.init_node("detect_human")
    node = Node()
    rospy.spin()
