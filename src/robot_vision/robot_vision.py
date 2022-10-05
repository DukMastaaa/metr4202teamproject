#!/usr/bin/python3
import rospy
from geometry_msgs.msg import Pose,Point
from fiducial_msgs.msg import FiducialTransformArray

import tf2_ros
from tf_conversions import transformations as tfct
import modern_robotics as mr
import numpy as np

# transformation matrix from the base frame to the camera frame
T_bc = np.array([
    [0, 1, 0, 0.20],
    [1, 0, 0, 0],
    [0, 0, -1, 0.445],
    [0,0,0,1]]
)

class RobotVision:
    NODE_NAME_TO_PUBLISH = "desired_pose"
    NODE_NAME_TO_SUBSCRIBE = "fiducial_transforms"

    def __init__(self):
        self.pub = rospy.Publisher(
            self.NODE_NAME_TO_PUBLISH,
            Pose,
            queue_size = 10
        )
        self.sub = rospy.Subscriber(
            self.NODE_NAME_TO_SUBSCRIBE,
            FiducialTransformArray,
            self.callback
        )


    def callback(self, fiducialtransform: FiducialTransformArray) -> None:
        fidtransform = fiducialtransform.transforms
        if len(fidtransform) >= 1:
            t_form = fidtransform[0].transform
            t_ct = t_form.translation
            q_ct = t_form.rotation

            # from week 5 prac
            p_ct = np.array([t_ct.x, t_ct.y, t_ct.z])
            R_ct = tfct.quaternion_matrix(np.array([q_ct.x, q_ct.y, q_ct.z, q_ct.w]))[:3, :3]
            T_ct = mr.RpToTrans(R_ct, p_ct)

            T_bt = T_bc @ T_ct

            R_bt, p_bt = mr.TransToRp(T_bt)
            #q_bt = tfct.quaternion_from_matrix(R_bt)

            msg = Pose(
                position = Point(p_bt[0], p_bt[1], p_bt[2])
            )

            self.pub.publish(msg)

def main():
    rospy.init_node("tag_detection", anonymous = True)

    td = RobotVision()

    rospy.spin()


if __name__ == '__main__':
    main()