#!/usr/bin/python3
import rospy
from sensor_msgs.msg import Pose
from fiducial_msgs.msg import FiducialTransform 
import tf2_ros
import modern_robotics as mr
import numpy as np


class TagDetection:
    NODE_NAME_TO_PUBLISH:"desired_pose"
    NODE_NAME_TO_SUBSCRIBE: "fiducial_transforms"

    def __init__(self):

        self.pub = rospy.Publisher(
            self.NODE_NAME_TO_PUBLISH,
            Pose,
            queue_size = 10
        )
        self.sub = rospy.Subscriber(
            self.NODE_NAME_TO_SUBSCRIBE,
            FiducialTransform,
            self.callback
        )

    def camera_rel_base():
        T = np.array([])
        return T


    def callback(self, fiducialtransform: FiducialTransform) -> None:
        Tbc = TagDetection.camera_rel_base()
        Tct = fiducialtransform

        Tbt = Tbc@Tct
        msg = Pose(

        )

        self.pub.publish(msg)



def main():
    rospy.init_node("tag_detection", anonymous = True)

    td = TagDetection()

    rospy.spin()


if __name__ == '__main__':
    main()