#! /usr/bin/env python3

import rospy
from std_msgs.msg import String

def callback(msg: String):
    # outputs a log every time it receives a message
    rospy.loginfo(rospy.get_caller_id() + f" \nReceived msg: {msg.data}")

def subscriber():
    rospy.init_node("listener_node", anonymous=True)
    rospy.Subscriber("talk_topic", String, callback)
    rospy.spin()

try:
    subscriber()
except rospy.ROSInterruptException:
    pass
