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
from std_msgs.msg import String
import math

import actionlib
from actionlib_msgs.msg import *
from geometry_msgs.msg import Pose, PoseWithCovarianceStamped, Point, Quaternion, Twist
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from tf.transformations import quaternion_from_euler

original = 0
start = 1

LOCATIONS = {
    # "office": (..., ...),
    "elevator": (10, -1.3),
    "toilet": (19, -3.2),
    "one": (25.7, 6.15),
    "two": (23, 8.43),
    "three": (24.8, 11.9),
    "hall": (-2, 1.2),
    "cafeteria": (26.6, -1.63),    
}
    

def cal_theta(x, y):
    return math.atan2(x, y)

def cal_location(x, y):
    theta = cal_theta(x, y)
    quaternion = quaternion_from_euler(0.0, 0.0, theta)
    
    return Pose(Point(x, y, 0.000), Quaternion(quaternion[0], quaternion[1], quaternion[2], quaternion[3]))

class NavToPoint:
    def __init__(self):
        
        rospy.Subscriber("bot/nav", String, self.callback)
        self.pub = rospy.Publisher("bot/reached", String, queue_size = 10)
        
        # Convert location.
        self.locations = {}
        for location in LOCATIONS:
            self.locations[location] = cal_location(
                LOCATIONS[location][0], LOCATIONS[location][1] # x, y
            )
        
	    # Subscribe to the move_base action server
        self.move_base = actionlib.SimpleActionClient("move_base", MoveBaseAction)
        rospy.on_shutdown(self.cleanup)

        rospy.loginfo("Waiting for move_base action server...")

        # Wait for the action server to become available
        self.move_base.wait_for_server(rospy.Duration(120))
        rospy.loginfo("Connected to move base server")

        # A variable to hold the initial pose of the robot to be set by the user in RViz
        initial_pose = PoseWithCovarianceStamped()
        rospy.Subscriber('initialpose', PoseWithCovarianceStamped, self.update_initial_pose)

	    # Get the initial pose from the user
        rospy.loginfo("*** Click the 2D Pose Estimate button in RViz to set the robot's initial pose...")
        rospy.wait_for_message('initialpose', PoseWithCovarianceStamped)
        
        # Make sure we have the initial pose
        while initial_pose.header.stamp == "":
            rospy.sleep(1)
            
        rospy.loginfo("Ready to go")
        rospy.sleep(1)
    
    def callback(self, data):
        self.move_base.cancel_goal()
        location = data.data.lower()
        
        if location in self.locations:           
            self.goal = MoveBaseGoal()
            rospy.loginfo("Starting navigation")

            while not rospy.is_shutdown():
                self.goal.target_pose.header.frame_id = 'map'
                self.goal.target_pose.header.stamp = rospy.Time.now()

                rospy.loginfo("Going to point "+ location)
                rospy.sleep(2)
                print(self.locations[location])
                self.goal.target_pose.pose = self.locations[location]
                self.move_base.send_goal(self.goal)
                waiting = self.move_base.wait_for_result(rospy.Duration(300))
                print(waiting)
                if waiting == 1:
                    rospy.loginfo("Reached point A")
                    rospy.sleep(2)
                    rospy.loginfo("Ready to go back")
                    rospy.sleep(2)
                    
                    # send back to main.
                    self.pub.publish(location)
                    return

                rospy.Rate(5).sleep()
        
        elif location == "stop":
            self.move_base.cancel_goal()

    def update_initial_pose(self, initial_pose):
        self.initial_pose = initial_pose
        self.origin = self.initial_pose.pose.pose

    def cleanup(self):
        rospy.loginfo("Shutting down navigation	....")
        self.move_base.cancel_goal()

if __name__=="__main__":
    rospy.init_node('navi_point')
    try:
        NavToPoint()
        rospy.spin()
    except Exception as e:
        print(e)
        pass
