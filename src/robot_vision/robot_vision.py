#!/usr/bin/python3
import rospy
from sensor_msgs.msg import Pose
from fiducial_msgs.msg import FiducialTransform 
import tf2_ros
from tf_conversions import transformations as tfct
import modern_robotics as mr
import numpy as np

# transformation matrix from the base frame to the camera frame
T_bc = np.array(
    [0, 1, 0, 0.16],
    [1, 0, 0, 0],
    [0, 0, -1, 0.445]
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
            FiducialTransform,
            self.callback
        )

    def callback(self, fiducialtransform: FiducialTransform) -> None:
        transform = fiducialtransform.transform
        t_ct = transform.translation
        q_ct = transform.rotation

        # from week 5 prac
        p_ct = np.array([t_ct.x, t_ct.y, t_ct.z])
        R_ct = tfct.quaternion_matrix(np.array([q_ct.x, q_ct.y, q_ct.z, q_ct.w]))[:3, :3]
        T_ct = mr.RpToTrans(R_ct, p_ct)

        T_bt = T_bc @ T_ct

        msg = Pose(
            
        )

        self.pub.publish(msg)

def main():
    rospy.init_node("tag_detection", anonymous = True)

    td = RobotVision()

    rospy.spin()


if __name__ == '__main__':
    main()