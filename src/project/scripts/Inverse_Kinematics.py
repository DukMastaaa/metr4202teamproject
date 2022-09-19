#!/usr/bin/env python3


# Always need this
from cmath import sqrt
import rospy

# Import message types
from std_msgs.msg import Header
from sensor_msgs.msg import JointState
from geometry_msgs.msg import Pose
import modern_robotics as mr
from math import cos,sin
import numpy as np
 


def inverse_kinematics(pose: Pose) -> JointState:
    global pub
    L1 = float
    L2 = float
    L3 = float
    L4 = float
    bx = pose.position.x - L4*cos(290)
    by = pose.position.y - L1 - L4*sin(290)
    bz = pose.position.z

    c3 = (bx^2 + by^2 - L2^2 - L3^2)
    s3 = sqrt(1-c3^2)

    theta_3 = mr.

    rospy.loginfo(f'Got desired pose\n[\n\tpos:\n{pose.position}\nrot:\n{pose.orientation}\n]')
    pub.publish(dummy_joint_states())
    


# Funny code
def dummy_joint_states() -> JointState:
    # Create message of type JointState
    msg = JointState(
        # Set header with current time
        header=Header(stamp=rospy.Time.now()),
        # Specify joint names (see `controller_config.yaml` under `dynamixel_interface/config`)
        name=['joint_1', 'joint_2', 'joint_3', 'joint_4']
    )
    # Funny code
    msg.position = [
        random.uniform(-1.5, 1.5),
        random.uniform(-1.5, 1.5),
        random.uniform(-1.5, 1.5),
        random.uniform(-1.5, 1.5)
    ]
    return msg


def main():
    global pub
    # Create publisher
    pub = rospy.Publisher(
        'desired_joint_states', # Topic name
        JointState, # Message type
        queue_size=10 # Topic size (optional)
    )

    # Create subscriber
    sub = rospy.Subscriber(
        'desired_pose', # Topic name
        Pose, # Message type
        inverse_kinematics # Callback function (required)
    )

    # Initialise node with any node name
    rospy.init_node('metr4202_w7_prac')

    # You spin me right round baby, right round...
    # Just stops Python from exiting and executes callbacks
    rospy.spin()


if __name__ == '__main__':
    main()
Footer
© 2022 GitHub, Inc.
Footer navigation
Terms
Privacy
Security
Status
Docs
Contact GitHub
Pricing
API
Training
Blog
About
