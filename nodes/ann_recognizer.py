#!/usr/bin/env python

# Software License Agreement (BSD License)
#
# Copyright (c) 2008, Willow Garage, Inc.
# All rights reserved.
#
## Simple talker demo that published std_msgs/Strings messages
## to the 'chatter' topic

"""
ann_recognizer.py is a MLP voice command recognizer
  publications:
    chatter (std_msgs/String) - text output
"""

import roslib; roslib.load_manifest('ann1vn')
import rospy
from std_msgs.msg import String

from record1 import record_to_file
from features import mfcc
from mlptester1 import *
import scipy.io.wavfile as wav

def ann_recognizer():
    pub = rospy.Publisher('chatter', String, queue_size=10)
    rospy.init_node('ann_recognizer', anonymous=True)
    rate = rospy.Rate(10) # 10hz

    words = ['backward','forward','go','left','right','stop']

    # MLP Network with 3 layers (in, hidden, out)
    mlp_layers = (260,25,6)

    filename_test = rospy.get_param("~test")
    filename_ann1 = rospy.get_param("~ann1")

    while not rospy.is_shutdown():
        rospy.loginfo("Record and Test")

        # Record to file
        record_to_file(filename_test)

        # Feed into MLP
        testNet = testInit(mlp_layers,filename_ann1)
        inputArray = extractFeature(filename_test)
        outStr = feedToNetwork(words,inputArray,testNet)

        # Publish output string
        rospy.loginfo(str(outStr))
        pub.publish(str(outStr))
        rate.sleep()

if __name__ == '__main__':
    try:
        ann_recognizer()
    except rospy.ROSInterruptException:
        pass
