#!/usr/bin/python3
import rospy
from std_msgs.msg import Header
from geometry_msgs.msg import Vector3, Transform, Quaternion, Pose, Point
from fiducial_msgs.msg import FiducialTransformArray, FiducialArray
from robot_msgs.msg import LuggageTransformArray, LuggageTransform

from tf_conversions import transformations
import modern_robotics as mr
import numpy as np

# transformation matrix from the base frame to the camera frame
T_bc = np.array([
    [0, 1, 0, 0.178],
    [1, 0, 0, 0],
    [0, 0, -1, 0.475-0.034-0.03],
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
    _, p = mr.TransToRp(T)
    p = Vector3(p[0], p[1], p[2])
    q = transformations.quaternion_from_matrix(T)
    q = Quaternion(x=q[0], y=q[1], z=q[2], w=q[3])
    return Transform(translation=p, rotation=q)

def vector_to_point(v: Vector3) -> Point:
    return Point(v.x, v.y, v.x)

def transform_to_pose(tf: Transform) -> Pose:
    return Pose(
        position=vector_to_point(tf.translation),
        orientation=tf.rotation
    )

def SE3_to_pose(T: np.array) -> Pose:
    _, p_vec = mr.TransToRp(T)
    p = Point(p_vec[0], p_vec[1], p_vec[2])
    q_vec = transformations.quaternion_from_matrix(T)
    q = Quaternion(x=q_vec[0], y=q_vec[1], z=q_vec[2], w=q_vec[3])
    return Pose(position=p, orientation=q)

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
            self.NODE_NAME_TO_SUBSCRIBE,
            FiducialTransformArray,
            self.callback
        )

    def callback(self, fid_tf_array: FiducialTransformArray) -> None:
        self.luggage_tf_array = []
        for fid_tf in fid_tf_array.transforms:
            tf_ct = fid_tf.transform
            T_ct = transform_to_SE3(tf_ct)
            T_bt = T_bc @ T_ct
            pose_bt = SE3_to_pose(T_bt)

            luggage = LuggageTransform(
                fiducial_id=fid_tf.fiducial_id,
                transform=pose_bt
            )
            self.luggage_tf_array.append(luggage)
            
        header = Header()
        header.stamp = rospy.Time.now()
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