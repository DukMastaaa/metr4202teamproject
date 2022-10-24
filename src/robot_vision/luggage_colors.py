#!/usr/bin/python3

# 0 Imports and global variables

# 0.1 Importing packages, classes and methods
import rospy
import numpy as np
import cv2
from std_msgs.msg import Header, ColorRGBA, Int32
from robot_msgs.msg import ColorWithID, ColorWithIDArray, LuggageColor, LuggageColorArray
from fiducial_msgs.msg import FiducialArray
from sensor_msgs.msg import Image
from robot_msgs.srv import DoColor, DoColorResponse
from cv_bridge import CvBridge, CvBridgeError

# 0.2 Establishing the RGB of the luggage colours.
red = ColorRGBA()
red.r = 255
red.g = 0
red.b = 0

green = ColorRGBA()
green.r = 0 
green.g = 255
green.b = 0

blue = ColorRGBA()
blue.r = 0
blue.g = 0
blue.b = 255

yellow = ColorRGBA()
yellow.r = 255
yellow.g = 255
yellow.b = 0

# 1 Luggage colour
class TagColour:
    SERIAL = 31700851

    def __init__(self):
        # Service for colour detection
        self.srv = rospy.Service(
            "do_color_detect",
            DoColor,
            self.image_callback
        )

        self.bridge = CvBridge()
        self.image_locked = False
        self.image_bgr = None
    
    def get_color(self, color) -> Int32:
        """
        Chooses the closest luggage colour of the inputted pixel RGB based on thresholds.
        """
        if color.r > 200:
            if color.g > 200:
                return(3) # Yellow block detected
            else:
                return(0) # Red block detected
        elif color.g > 200:
            return(1) # Green block detected
        elif color.b > 100:
            return(2) # Blue block detected
        else:
            return(4) # Non-standard color detected

    def image_callback(self,req):
        """
        Finds the middle pixel colour and delivers the result.
        """
        
        if req:
            data = rospy.wait_for_message(f"/ximea_ros/ximea_{self.SERIAL}/image_raw",Image)
            global img
            try:
                img = self.bridge.imgmsg_to_cv2(data,"bgr8")
            except CvBridgeError as e:
                print(e)
            #Get middle pixel

            bgr = img[img.shape[0]//2,img.shape[1]//2,:]
            color = ColorRGBA()
            color.r = bgr[2]
            color.g = bgr[1]
            color.b = bgr[0]
            detected_color = self.get_color(color)
            print(detected_color)
            msg = Int32(data=detected_color)
        
            return DoColorResponse(msg)

if __name__ == "__main__":
    rospy.init_node("luggage_colors", anonymous = True)
    tc = TagColour()
    rospy.spin()
