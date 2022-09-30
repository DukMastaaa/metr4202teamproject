#!/usr/bin/python3

import rospy
import modern_robotics as mr

from std_msgs.msg import Header
from sensor_msgs.msg import JointState
from geometry_msgs.msg import Pose

from math import atan, atan2, cos, sin, sqrt, pi
import numpy as np

import constants

from typing import Tuple

class InverseKinematics:
    NODE_NAME_TO_PUBLISH = "desired_joint_states"
    NODE_NAME_TO_SUBSCRIBE = "desired_pose"
    
    def __init__(self):
        self.pub = rospy.Publisher(
            self.NODE_NAME_TO_PUBLISH,
            JointState,
            queue_size=10
        )
        self.sub = rospy.Subscriber(
            self.NODE_NAME_TO_SUBSCRIBE,
            Pose,
            self.inverse_kinematics
        )
    
    @staticmethod
    def solution_2r(L1, L2, px, py) -> Tuple[float, float]:
        """
        Calculates the elbow-up IK solution for a 2R robot
        with lengths L1 and L2, and desired position (px, py).
        """
        c2 = (px**2 + py**2 - L1**2 - L2**2) / (2 * L1 * L2)
        if abs(c2) >= 1:
            raise ValueError("c2 >= 1, point out of range")
        theta2 = atan2(-sqrt(1 - c2**2), c2)
        theta1 = atan2(py, px) - atan2(L2 * sin(theta2), L1 + L2 * cos(theta2))
        return theta1, theta2
    
    @staticmethod
    def solution_4r(L1, L2, L3, L4, theta_e, px, py, pz) -> Tuple[float, float, float, float]:
        """
        Calculates the elbow-up IK solution for our robot with lengths
        defined in constants.py, desired position (px, py, pz),
        and desired end-effector angle theta_e measured down from horizontal.
        """
        # We want to reduce the 4R arm to a 2R arm in (r, z) coordinates, where
        # the start of the 2R arm is L1 above the first joint, and 
        # the end is the position of joint 4 relative to the start.
        br = sqrt(px**2 + py**2) - L4 * cos(theta_e)
        phi = atan2(py, px)
        bz = pz + L4 * sin(theta_e) - L1
        
        # Next, we calculate the joint angles for the 2R arm,
        # where positive angles move the arm up.
        theta_a, theta_b = InverseKinematics.solution_2r(L2, L3, br, bz)

        # Finally, as our theta_i are measured from different points,
        # we do some conversion from theta_a, theta_b to theta_2, theta_3,
        # and calculate the rest of the angles.
        theta_1 = phi
        theta_2 = np.deg2rad(90) - theta_a
        theta_3 = -theta_b
        theta_4 = theta_e - theta_2 - theta_3 + np.deg2rad(90)

        return theta_1, theta_2, theta_3, theta_4

    @staticmethod
    def compensated_angles(theta_1, theta_2, theta_3, theta_4) -> Tuple[float, float, float, float]:
        """
        Returns the angles (angle_1, angle_2, angle_3, angle_4)
        which are meant for the actual Dynamixels with IDs 1, 2, 3, 4.
        This is necessary as the way we assembeled the motors isn't
        consistent with the definitions of theta for IK.
        """
        # compensate for motor orientation
        theta_2 = np.deg2rad(90) - theta_2
        theta_3 = -theta_3
        # compensate for different motor IDs
        # angle_i is the angle that the motor with ID i should recieve
        angle_1 = theta_4
        angle_2 = theta_1
        angle_3 = theta_2
        angle_4 = theta_3
        return angle_1, angle_2, angle_3, angle_4

    def callback(self, pose: Pose) -> None:
        """
        Callback for the subscriber, which calculates the desired joint angles,
        compensates for inconsistencies in motor assembly, and publishes
        to the next stage.
        """
        theta_1, theta_2, theta_3, theta_4 = self.solution_4r(
            constants.L1, constants.L2, constants.L3, constants.L4,
            np.deg2rad(constants.THETA_E_DEG),
            pose.position.x, pose.position.y, pose.position.z
        )
        
        angle_1, angle_2, angle_3, angle_4 = self.compensated_angles(
            theta_1, theta_2, theta_3, theta_4
        )
        
        msg = JointState(
            # Set header with current time
            header=Header(stamp=rospy.Time.now()),
            # Specify joint names (see `controller_config.yaml` under `dynamixel_interface/config`)
            name=['joint_1', 'joint_2', 'joint_3', 'joint_4'],
            position=[angle_1, angle_2, angle_3, angle_4]
        )
        
        self.pub.publish(msg)

def main():
    # Initialise node with any node name
    rospy.init_node('inverse_kinematics', anonymous = True)
    
    # initialise IK class
    ik = InverseKinematics()
    
    # You spin me right round baby, right round...
    # Just stops Python from exiting and executes callbacks
    rospy.spin()


if __name__ == '__main__':
    main()
