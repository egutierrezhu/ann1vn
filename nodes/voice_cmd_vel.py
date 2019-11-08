#!/usr/bin/env python

"""
  voice_cmd_vel.py is a simple demo of speech recognition.
  You can control a mobile base using commands found
  in the corpus file.
"""

import roslib; roslib.load_manifest('ann1vn')
import rospy
import math

from geometry_msgs.msg import Twist
from std_msgs.msg import String
from math import copysign

class voice_cmd_vel:

    def __init__(self):
        rospy.on_shutdown(self.cleanup)
        
        # Set a number of parameters affecting the robot's speed     
        self.speed = rospy.get_param("~start_speed", 0.1)
        self.angular_speed = rospy.get_param("~start_angular_speed", 0.2)
        self.angular_increment = rospy.get_param("~angular_increment", 0.1)
        
        # We don't have to run the script very fast
        self.rate = rospy.get_param("~rate", 5)
        r = rospy.Rate(self.rate)

        # Time, in seconds, for waiting correct command
        self.wait_time_initial = 5
        self.wait_time = self.wait_time_initial

        # A flag to determine whether or not TIAGo voice control
        self.TIAGo = False
        
        # Initialize the Twist message we will publish.
        self.cmd_vel = Twist()

        # Publish the Twist message to the cmd_vel topic
        self.cmd_vel_pub = rospy.Publisher('cmd_vel', Twist, queue_size=5)
        
        # Subscribe to the /ann_recognizer/output topic to receive voice commands.
        rospy.Subscriber("chatter", String, self.speech_callback)

        
        # A mapping from keywords or phrases to commands
        self.keywords_to_command = {'backward': ['backward'],
                                    'forward': ['forward'],
                                    'turn left': ['left'],
                                    'turn right': ['right']}
        
        rospy.loginfo("Ready to receive voice commands")
        
        # We have to keep publishing the cmd_vel message if we want the robot to keep moving.
        while not rospy.is_shutdown():
            self.cmd_vel_pub.publish(self.cmd_vel)
            r.sleep()                       
            
    def get_command(self, data):
        # Attempt to match the recognized word or phrase to the 
        # keywords_to_command dictionary and return the appropriate
        # command
        for (command, keywords) in self.keywords_to_command.iteritems():
            for word in keywords:
                if data.find(word) > -1:
                    # Wait time for other command, in seconds
                    self.time1 = rospy.get_rostime()
                    self.wait_time = 10
                    return command
        
    def speech_callback(self, msg):
	rospy.loginfo('Moving the base through velocity commands')

        # If the user said TIAGo, set the flag as true 
        if msg.data == 'go':
            if self.TIAGo:
                # Stop the robot!
                rospy.loginfo("Command: " + str("stop, go canceled"))
                self.cmd_vel = Twist()
                self.TIAGo = False
                self.wait_time = self.wait_time_initial
                return
            else:
                rospy.loginfo("TIAGo waiting command")
                self.TIAGo = True
                self.time1 = rospy.get_rostime()
                #rospy.loginfo("Current time1 %i", self.time1.secs)
                return
        elif msg.data == 'stop':
            # Stop the robot!  Publish a Twist message consisting of all zeros. 
            rospy.loginfo("Command: " + str(msg.data))
            self.cmd_vel = Twist()
            self.TIAGo = False
            self.wait_time = self.wait_time_initial
            return

        # If TIAGo voice control is true and not out of time
        # Get the motion command from the recognized phrase
        # If false, simply return without performing any action
        if self.TIAGo:
            self.time2 = rospy.get_rostime()
            #rospy.loginfo("Current time2 %i", self.time2.secs)
            if self.time2.secs < self.time1.secs+self.wait_time:
                command = self.get_command(msg.data)
            else:
                self.TIAGo = False
                self.wait_time = self.wait_time_initial
                return
        else:
            return

        # Log the command to the screen
        rospy.loginfo("Command: " + str(command))
             
        
        # The list of if-then statements should be fairly
        # self-explanatory
        if command == 'backward':
            self.cmd_vel.linear.x = -self.speed
            self.cmd_vel.angular.z = 0

        elif command == 'forward':    
            self.cmd_vel.linear.x = self.speed
            self.cmd_vel.angular.z = 0
            
        elif command == 'turn left':
            if self.cmd_vel.linear.x != 0:
                if self.cmd_vel.angular.z < self.angular_speed:
                    self.cmd_vel.angular.z += self.angular_increment
            else:        
                self.cmd_vel.angular.z = self.angular_speed
                
        elif command == 'turn right':    
            if self.cmd_vel.linear.x != 0:
                if self.cmd_vel.angular.z > -self.angular_speed:
                    self.cmd_vel.angular.z -= self.angular_increment
            else:        
                self.cmd_vel.angular.z = -self.angular_speed
                
        else:
            return

    def cleanup(self):
        # When shutting down be sure to stop the robot!
        twist = Twist()
        self.cmd_vel_pub.publish(twist)
        rospy.sleep(1)

if __name__=="__main__":
    rospy.init_node('voice_cmd_vel')
    try:
        voice_cmd_vel()
        # spin() simply keeps python from exiting
	# until this node is stopped
        rospy.spin()
    except rospy.ROSInterruptException:
        rospy.loginfo("Moving the base through velocity commands terminated.")

