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

import time

TIMEOUT = 60 # seconds

### Nodes
# reg
# output topic: bot/voice_input

# speak
# input topic: bot/speak

# camera - never use
# input topic: bot/take_photo

# detect chair
# input topic: bot/start_object_dec
# output topic: bot/found_object

# detect human
# input topic: bot/start_human_dec
# output topic: bot/found_human

# nav
# input topic: bot/nav
# output topic: bot/reached


class CustomNode:
    def __init__(self):
        rospy.init_node("bot_main")
        
        # Google Voice recognizer, Google TTS
        self.reg_sub = rospy.Subscriber("bot/voice_input", String, self.heard)
        self.speak_pub = rospy.Publisher("bot/speak", String, queue_size = 1)
        
        # Navigation
        self.nav_sub = rospy.Subscriber("bot/reached", String, self.reached)
        self.nav_pub = rospy.Publisher("bot/nav", String, queue_size = 2)
        
        # Human detection
        self.hum_sub = rospy.Subscriber("bot/found_human", String, self.found_hum)
        self.hum_pub = rospy.Publisher("bot/start_human_dec", String, queue_size = 10)
        
        # Object detection
        self.obj_sub = rospy.Subscriber("bot/found_object", String, self.found_obc)
        self.obj_pub = rospy.Publisher("bot/start_object_dec", String, queue_size = 10)
        
        # Bot state.
        self.state = "NONE"
        
        self.reset_state()
        
        self.speak("Starting program...")
        
        rospy.sleep(5)
        
        while not rospy.is_shutdown():
            self.update_state()
            rospy.sleep(1)
    
    def reset_state(self):
        self.is_reached = False
        self.reached_location = ""
        self.going_to = ""
        
        self.found_human = False
        self.found_object = False
        
        self.is_listening = False
        self.recognized = False        
        self.start_time = None
        self.message = ""
    
    def update_state(self):
        
        # Init
        if self.state == "NONE":
            
            # At elevator, waiting...
            if self.is_reached:
                self.speak("Waiting for human!")

                rospy.sleep(4)
                self.act_hum_dec()
                self.state = "IDLE"
                
            # Return back to elevator.
            elif self.going_to != "elevator":
                self.speak("Im going to go to the elevator.")
                self.is_reached = False # TODO: 

                rospy.sleep(4)
                self.goto("elevator")
        
        # Idle at elevator.
        elif self.state == "IDLE":
            
            # Somebody came.
            if self.found_human:
                self.speak("Welcome to the hospital!")
                
                # TODO: move arm.
                
                rospy.sleep(5)
                
                self.speak("Where do you want to go?")

                rospy.sleep(6)
                
                self.is_listening = True
                self.start_time = time.time()
                self.state = "WAIT-FOR-RESPOND"
        
        elif self.state == "WAIT-FOR-RESPOND":
            rn = time.time()
            
            # Heard humans.
            if self.recognized:
                self.recognized = False # Reset
                
                msg = self.message.replace("tree", "three").replace("pilot", "toilet") # Make it easier to type
                self.start_time = time.time()

                if msg in ["hello", "hey", "hi"]:
                    self.speak("Hello, Where do you want to go?")
                    
                    rospy.sleep(4)
                
                elif msg in ["cafeteria", "hall", "toilet"]:
                    self.is_reached = False
                    
                    self.speak("So you want to go to " + msg)
                    
                    rospy.sleep(5)
                    
                    self.speak("Please follow me!")  
                                      
                    self.goto(msg)
                    self.state = "GUILDING"
                
                elif msg in ["one", "two", "three"]:
                    self.is_reached = False
                        
                    self.speak("So you have an appointment?")
                    
                    rospy.sleep(3)
                    
                    self.speak("Let's go to room " + msg + " then. Follow me!")
                    
                    rospy.sleep(5)
                    
                    self.goto(msg)
                    self.state = "GUILDING"
                
                else:
                    self.speak("I do not understand what you meant. Please say where you want to go!")
                    
                    rospy.sleep(8)

                self.is_listening = True
                    
                    
            # Didn't hear shit. :(
            elif rn - self.start_time > TIMEOUT:
                self.speak("Nobody answered me. I am going to reset myself!")
                
                self.reset_state()
                self.state = "NONE"

        elif self.state == "GUILDING":
            # Dynamic user input
            self.is_listening = True
            
            # Reached target.
            if self.is_reached:
                self.is_listening = False
                self.recognized = False
                self.is_reached = False
                
                self.speak("Hey we are finally here!")
                
                rospy.sleep(3)
                
                if self.reached_location in ["one", "two", "three"]:
                    self.found_object = False
                    self.act_obj_dec()
                    self.state = "FINDING-CHAIR"
                elif self.reached_location == "elevator":
                    self.speak("Good bye! Have a nice day!")
                    
                    rospy.sleep(4)
                    
                    self.reset_state()
                    self.state = "NONE"
                else:
                    self.speak("Do you want me to stay here with you?")
                    
                    self.is_listening = True
                    self.recognized = False
                    
                    rospy.sleep(2)
                    
                    self.start_time = time.time()
                    self.state = "YES-OR-RESET"
            
            if self.recognized:
                self.recognized = False
                msg = self.message.replace("tree", "three")

                if "hall" in msg:
                    self.nav_pub.publish("stop")
                    self.is_reached = False
                    
                    self.speak("You changed your mind huh? Let's go to the hall then!")
                    
                    rospy.sleep(6)
                    
                    self.goto("hall")

                elif "cafe" in msg:
                    self.nav_pub.publish("stop")
                    self.is_reached = False

                    self.speak("Ok, maybe you're hungry. Then follow me!")

                    rospy.sleep(5)

                    self.goto("cafeteria")
                
                elif "toilet" in msg or "pilot" in msg:
                    self.nav_pub.publish("stop")
                    self.is_reached = False
                    
                    self.speak("Maybe you're feeling it. Then follow me to the toilet!")
                    
                    rospy.sleep(6)
                    
                    self.goto("toilet")
                
                elif "elevator" in msg:
                    self.nav_pub.publish("stop")
                    self.is_reached = False
                    
                    self.speak("Are you going to leave now? Ok then. Follow me!")
                    
                    rospy.sleep(6)
                    
                    self.goto("elevator")
                
                elif "one" in msg:
                    self.nav_pub.publish("stop")
                    self.is_reached = False
                    
                    self.speak("You're changing to room one? Well okay.")
                    
                    rospy.sleep(8)
                    
                    self.speak("Please follow me!")
                    self.goto("one")
                    
                elif "two" in msg:
                    self.nav_pub.publish("stop")
                    self.is_reached = False
                    
                    self.speak("You're changing to room two? The doctor is actually great!")
                    
                    rospy.sleep(8)
                    
                    self.speak("Please follow me!")
                    self.goto("two")
                
                elif "three" in msg:
                    self.nav_pub.publish("stop")
                    self.is_reached = False
                    
                    self.speak("You're changing to room three? Well good luck.")
                    
                    rospy.sleep(8)
                    
                    self.speak("Please follow me!")
                    self.goto("three")
                
                elif "hey" in msg:
                    self.nav_pub.publish("stop")

                    self.speak("Debug mode activated")
                    

                    rospy.sleep(4)

                    self.is_reached = True
                    self.reached_location = "one"

                self.is_listening = True

        elif self.state == "FINDING-CHAIR":
            self.is_listening = False
            rospy.sleep(1)

            if self.found_object:
                rospy.loginfo("Found a chair.")
                self.speak("You can sit in that chair while waiting for the appointment!")
                
                rospy.sleep(8)
                
            self.speak("Do you want me to stay here with you?")
                
            rospy.sleep(4)
            
            self.recognized = False
            self.is_listening = True
            
            self.start_time = time.time()
            self.state = "YES-OR-RESET"
            
        elif self.state == "YES-OR-RESET":
            rn = time.time()
            
            if rn - self.start_time > TIMEOUT:
                self.speak("Welp, You said nothing. So good luck!")
                
                rospy.sleep(5)
                
                self.reset_state()
                self.state = "NONE"
            
            elif self.recognized:
                msg = self.message
                self.recognized = False
                
                if msg.startswith("y"):
                    self.speak("Ok, I will wait here for you.")
                    
                    rospy.sleep(20)

                    self.is_listening = True
                    self.recognized = False
                    self.is_reached = False
                    self.found_human = False

                    self.act_hum_dec()

                    self.state = "WAIT-UNTIL-RETURN"
                elif msg.startswith("n"):
                    self.speak("Then I will go back to the elevator, Have a nice day!")
                
                    rospy.sleep(5)
                    
                    self.reset_state()
                    self.state = "NONE"
                    
                
        elif self.state == "WAIT-UNTIL-RETURN":
            
            if self.found_human:
                self.speak("Well, let's get going.")
                
                self.goto("elevator")
                self.state = "GUILDING"
                
            elif self.recognized:
                self.recognized = False
                
                msg = self.message
                
                if "back" in msg:
                    self.speak("Oh you're back, Let's go then.")
                    
                    self.goto("elevator")
                    self.state = "GUILDING"
            
    
    def act_obj_dec(self):
        rospy.loginfo("Activating object detection")
        self.obj_pub.publish("activate")

    def act_hum_dec(self):
        rospy.loginfo("Activating human detection")   
        self.hum_pub.publish("activate")

    def found_obc(self, data):
        rospy.loginfo("Found obj")
        self.found_object = True
    
    def found_hum(self, data):
        rospy.loginfo("Found human")
        self.found_human = True
    
    def reached(self, data):
        rospy.loginfo("Reached " + data.data)
        self.is_reached = True
        self.reached_location = data.data
    
    def goto(self, location):
        rospy.loginfo("Navigating to " + location)    
        self.going_to = location    
        self.nav_pub.publish(location)
    
    def heard(self, data):
        if not self.is_listening:
            print("Ignoring input", data.data)
            return
        
        self.message = data.data.lower().replace("1", "one").replace("2", "two").replace("3", "three")  
        rospy.loginfo("Heard " + self.message)
        
        self.recognized = True     
        self.is_listening = False
    
    def speak(self, text):
        # Will have to disable listening while speaking or the bot will hear itself.
        self.is_listening = False
        self.recognized = False
        
        rospy.loginfo("Speaking " + text)
        self.speak_pub.publish(text)


if __name__ == '__main__':
    csNode = CustomNode()    
    rospy.spin()