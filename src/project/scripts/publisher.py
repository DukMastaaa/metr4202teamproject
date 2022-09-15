#! /usr/bin/env python3

import rospy
from std_msgs.msg import String

def string_pub():
    pub = rospy.Publisher("talk_topic", String, queue_size=10)

    # anonymous=True indicates unique name
    rospy.init_node("talker_node", anonymous=True)  # initialise code
    
    rate = rospy.Rate(10)  # rate at which ros sleeps, 10Hz
    
    # checks if the program should shut down
    while not rospy.is_shutdown():
        msg_str = f"the time is now {rospy.get_time()}"
        rospy.loginfo(msg_str)  # log message string to alert when it has been sent
        
        # initialise message class
        msg = String()
        # string class stores 1 string in data variable
        # in the tut, `rosmsg info std_msgs/` then tab
        # also, specifically for string, `rosmsg info std_msgs/String`
        msg.data = msg_str
        pub.publish(msg)
        rate.sleep()

try:
    string_pub()
except rospy.ROSInterruptException:
    pass
