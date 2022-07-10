#!/usr/bin/env python

"""

    RoboCup@Home Education | oc@robocupathomeedu.org
    navigation2.py - navigation primitives using move_base action

"""

# NOTE: Didn't have time to fix the nav node, so we used this example
#       and edited it instaed.

import math
import rospy

from std_msgs.msg import String

import actionlib
from actionlib_msgs.msg import *
from geometry_msgs.msg import Pose, Point, Quaternion
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from tf.transformations import quaternion_from_euler

ACTION_MOVE_BASE = 'move_base'
FRAME_map = 'map'
FRAME_base = 'base_frame'

LOCATIONS = {
    # "location_name": (x, y),
    # "office": (..., ...),
    "elevator": (10, -1.3),
    "toilet": (19, -3.2),
    "one": (25.7, 6.15),
    "two": (23, 8.43),
    "three": (24.8, 11.9),
    "hall": (-2, 1.2),
    "cafeteria": (26.6, -1.63),    
}

class NavToPoint:
    def __init__(self):
        rospy.on_shutdown(self.cleanup)
        
        # Subscribe to the move_base action server
        self.move_base = actionlib.SimpleActionClient(ACTION_MOVE_BASE, MoveBaseAction)

        rospy.loginfo("Waiting for move_base action server...")

        # Wait for the action server to become available
        self.move_base.wait_for_server(rospy.Duration(120))
        rospy.loginfo("Connected to move base server")

        self.map_robot_pose = [0,0,0]
        self.move_base_running = False
        self.blocking = True

        rospy.loginfo("Ready to go")
        rospy.sleep(1)

        rospy.Subscriber("bot/nav", String, self.callback)
        
        self.pub = rospy.Publisher("bot/reached", String, queue_size = 1)
    
    def callback(self, data):
        if data.data == "stop":
            self.stop()
        else:
            self.goto(data.data)

    # seng the goal and if blocking, then wait for termination 
    def goto(self, target, blocking=True):
        # Only move if target is registered in LOCATIONS.
        if target in LOCATIONS:
            _target = target # Save the location name in another variable
            target = LOCATIONS[target]
        
            target = (target[0], target[1], 0)

            self.blocking = blocking
            yaw = target[2]/180.0*math.pi
            quaternion = quaternion_from_euler(0.0, 0.0, yaw)
            targetpose = Pose(Point(target[0], target[1], 0.000), Quaternion(quaternion[0], quaternion[1], quaternion[2], quaternion[3]))

            self.goal = MoveBaseGoal()

            #while not rospy.is_shutdown():
            self.goal.target_pose.header.frame_id = 'map'
            self.goal.target_pose.header.stamp = rospy.Time.now()

            # Robot will go to target
            rospy.loginfo("Going to target %r" %target)
            rospy.sleep(2)
            self.goal.target_pose.pose = targetpose
            self.move_base.send_goal(self.goal)
            self.move_base_running = True
            rospy.sleep(0.2) # wait for action to actually start

            if blocking:
                # wait and check the result
                waiting = self.move_base.wait_for_result() 

                # close the action and print the result
                self.move_base_running = False

                if not waiting:
                    rospy.logerr("Action server not available!")
                else:
                    rospy.loginfo("Navigation result: %s" % self.move_base.get_result())
                    self.pub.publish(_target)


    # return current robot pose in map coordinates
    def get_robot_pose(self):
        try:
            (trans,rot) = listener.lookupTransform(FRAME_map, FRAME_base, rospy.Time(0))
        except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException) as e:
            print(e)
            return

        roll, pitch, yaw = tf.transformations.euler_from_quaternion(rot)
        self.map_robot_pose[0] = trans[0]
        self.map_robot_pose[1] = trans[1]
        self.map_robot_pose[2] = yaw
        return self.map_robot_pose


    def update_initial_pose(self, initial_pose):
        self.initial_pose = get_robot_pose()


    def stop(self):
        rospy.loginfo("!!! STOP !!!")
        if (self.move_base == None):
            self.move_base = actionlib.SimpleActionClient('move_base',MoveBaseAction)
        self.move_base.wait_for_server()
        self.move_base.cancel_all_goals()
        self.blocking = True
        self.move_base_running = False


    def cleanup(self):
        rospy.loginfo("Shutting down navigation ....")
        if self.blocking:
            self.move_base.cancel_goal()




if __name__=="__main__":
    rospy.init_node('navtopoint')
    nav = NavToPoint()
    rospy.spin()