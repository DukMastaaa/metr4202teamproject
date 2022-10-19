#!/usr/bin/python3
import rospy
from std_msgs.msg import Header
from geometry_msgs.msg import Transform
from fiducial_msgs.msg import FiducialTransformArray, FiducialArray
from robot_msgs.msg import LuggageTransformArray, LuggageTransform

from tf_conversions import transformations
import modern_robotics as mr
import numpy as np

# transformation matrix from the base frame to the camera frame
T_bc = np.array([
    [0, 1, 0, 0.20],
    [1, 0, 0, 0],
    [0, 0, -1, 0.445],
    [0,0,0,1]]
)

def transform_to_SE3(transform: Transform) -> np.array:
    # from week 5 prac
    t = transform.translation
    q = transform.rotation
    p = np.array([t.x+0.0017, t.y+0.0017, t.z])
    R = transformations.quaternion_matrix(np.array([q.x, q.y, q.z, q.w]))[:3, :3]
    T = mr.RpToTrans(R, p)
    return T

def SE3_to_transform(T: np.array) -> Transform:
    R, p = mr.TransToRp(T)
    q = transformations.quaternion_from_matrix(R)
    return Transform(translation=p, rotation=q)


class RobotVision:
    NODE_NAME_TO_PUBLISH = "luggage_transforms"
    NODE_NAME_TO_SUBSCRIBE = "fiducial_transforms"

    def __init__(self):
        self.luggage_info_pub = rospy.Publisher(
            self.NODE_NAME_TO_PUBLISH,
            LuggageTransformArray,
            queue_size = 10
        )
        self.transform_sub = rospy.Subscriber(
            self.NLODE_NAME_TO_SUBSCRIBE,
            FiducialTransformArray,
            self.callback
        )

    def callback(self, fid_tf_array: FiducialTransformArray) -> None:
        self.luggage_tf_array = []
        for fid_tf in fid_tf_array.transforms:
            tf_ct = fid_tf.transform
            T_ct = transform_to_SE3(tf_ct)
            T_bt = T_bc @ T_ct
            tf_bt = SE3_to_transform(T_bt)

            luggage = LuggageTransform(
                id=fid_tf.id,
                transform=tf_bt
            )
            self.luggage_tf_array.append(luggage)
            
        header = Header()
        header.stamp = rospy.time.now()
        msg = LuggageTransformArray(
            header=header,
            transforms=self.luggage_tf_array
        )
        self.luggage_info_pub.publish(msg)

def main():
    rospy.init_node("tag_detection", anonymous = True)

    td = RobotVision()

    rospy.spin()


if __name__ == '__main__':
    main()