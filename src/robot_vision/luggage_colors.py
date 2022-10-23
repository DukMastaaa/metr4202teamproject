#!/usr/bin/python3
from tokenize import Int32
import rospy
import numpy as np
import cv2
from std_msgs.msg import Header, ColorRGBA
from robot_msgs.msg import ColorWithID, ColorWithIDArray, LuggageColor, LuggageColorArray
from fiducial_msgs.msg import FiducialArray
from sensor_msgs.msg import Image

from cv_bridge import CvBridge, CvBridgeError

# sdflksjdflkjs
#Establish Red Blue Green 
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

class TagColour:
    SERIAL = 31700851

    def __init__(self):
        self.image_sub = rospy.Subscriber(
            f"/ximea_ros/ximea_{self.SERIAL}/image_raw",
            Image,
            self.image_callback
        )
        self.color_pub = rospy.Publisher(
            "luggage_colors",
            LuggageColorArray,
            queue_size = 10
        )

        self.bridge = CvBridge()
        self.image_locked = False
        self.image_bgr = None
    
    def get_color(self, color) -> Int32:
        # min_dis = 100
        # for solid_color in [red,green,blue,yellow]:
        #     euclidian_distance = np.sqrt(np.sqrt(color.r-solid_color.r)+np.sqrt(color.g-solid_color.g)+np.sqrt(color.b-solid_color.b))
        #     if euclidian_distance < min_dis:
        #         min_dis = euclidian_distance
        #         return_color = solid_color

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

    def image_callback(self,data):
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
        self.color_pub.publish(detected_color)

if _name_ == "_main_":
    rospy.init_node("luggage_colors", anonymous = True)
    tc = TagColour()
    rospy.spin()
