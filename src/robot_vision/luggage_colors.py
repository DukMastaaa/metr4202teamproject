#!/usr/bin/python3
import rospy
import cv2
from std_msgs.msg import Header, ColorRGBA
from robot_msgs.msg import ColorWithID, ColorWithIDArray, LuggageColor, LuggageColorArray
from fiducial_msgs.msg import FiducialArray
from sensor_msgs.msg import Image

from cv_bridge import CvBridge, CvBridgeError

class TagColour:
    SERIAL = 31700851

    def __init__(self):
        self.vertices_sub = rospy.Subscriber(
            "fid_vertices",
            FiducialArray,
            self.vertices_callback
        )
        self.image_sub = rospy.Subscriber(
            f"/ximea_ros/ximea_{self.SERIAL}/image_raw",
            Image,
            self.image_callback
        )
        self.color_pub = rospy.Publisher(
            "test_color",
            ColorWithID,
            queue_size = 10
        )

        self.bridge = CvBridge()
        self.image_locked = False
        self.image_bgr = None
    
    def get_average_color(x0, y0, x1, y1, x2, y2, x3, y3) -> ColorRGBA:
        vert_0 = [x0, y0]
        vert_1 = [x1, y1]
        vert_2 = [x2, y2]
        vert_3 = [x3, y3]

        # stuff goes here, refer to self.image_bgr

        # for now
        return ColorRGBA(r=255, g=0, b=0, a=255)

    def image_callback(self, data):
        # adapted from Miguel's example_camera.py code.
        if not self.image_locked:
            try:
                self.image_bgr = self.bridge.imgmsg_to_cv2(data, "bgr8")
            except CvBridgeError as e:
                print(e)

    def vertices_callback(self, fid_array: FiducialArray) -> None:
        self.image_locked = True
        colors = []
        for fid in fid_array.fids:
            average_color = self.get_average_color(
                fid.x0, fid.y0, fid.x1, fid.y1, fid.x2, fid.y2, fid.x3, fid.y3
            )
            self.luggage_color_dict[fid.fiducial_id] = None
            color_with_id = ColorWithID(id=fid.fiducial_id, color=average_color)
            colors.append(color_with_id)
        
        header = Header()
        header.stamp = rospy.time.now()
        colors = list(self.luggage_color_dict.values())
        msg = ColorWithIDArray(
            header=header,
            colors=colors
        )
        self.color_pub.publish(msg)
        self.image_locked = False
        
