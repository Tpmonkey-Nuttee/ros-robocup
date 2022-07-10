#!/usr/bin/env python
import rospy
from robot_vision_msgs.msg import BoundingBoxes
from std_msgs.msg import String

class Node:
    def __init__(self):

        self.enable = False

        rospy.Subscriber("bot/start_object_dec", String, self.callenable)
        rospy.Subscriber("yolo_ros/bounding_boxes", BoundingBoxes, self.callback)
        self.pub = rospy.Publisher("bot/found_object", String, queue_size=10)

    def callback(self, data):
        if not self.enable:
            return
        else:
            for bounding_box in data.bounding_boxes:
                # print(data.bounding_boxes)
                if bounding_box.Class == 'chair':
                    self.enable = False
                    self.pub.publish("FoundChair")
                    break
                # xo = (bounding_box.xmin+bounding_box.xmax)//2
                # yo = (bounding_box.ymin+bounding_box.ymax)//2
                # 	print xo,' ',yo 

    def callenable(self,data):
        self.enable = True


if __name__ == '__main__':
    rospy.init_node("detect_object")
    node = Node()
    rospy.spin()
