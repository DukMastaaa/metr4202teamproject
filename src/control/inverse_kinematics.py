#!/usr/bin/python3


# Always need this
import rospy

# Import message types
from std_msgs.msg import Header
from sensor_msgs.msg import JointState
from geometry_msgs.msg import Pose
import modern_robotics as mr
from math import atan, atan2, cos,sin,sqrt,pi
import numpy as np
 


def inverse_kinematics(pose: Pose) -> JointState:
    global pub
    L1 = 0.053
    L2 = 0.12
    L3 = 0.095
    L4 = 0.113
    bx = pose.position.x - L4*cos(290*pi/180)
    by = pose.position.y - L1 - L4*sin(290*pi/180)
    bz = pose.position.z

    theta_1 = atan(bz/bx)
    c3 = (bx**2 + by**2 - L2**2 - L3**2)/(2*L2*L3)
    s3 = sqrt(1-c3**2)

    theta_3 = atan2(c3,s3)

    #s2 = (by*(L2 + L3*cos(theta_3))-L3*sin(theta_3)*bx)/(L2**2+L3**2 + 2*L2*L3*cos(theta_3))
    #c2 = (bx*(L2 + L3*cos(theta_3)) + L3*sin(theta_3)*by)/(L2**2+L3**2 + 2*L2*L3*cos(theta_3))

    #theta_2 = atan2(c2,s2)
    theta_2  = atan2(bx,by) - atan2((L2+L3*cos(theta_3)),L3*sin(theta_3))

    theta_4 = 290*pi/180 - theta_2 - theta_3

    
    # Create message of type JointState
    msg = JointState(
        # Set header with current time
        header=Header(stamp=rospy.Time.now()),
        # Specify joint names (see `controller_config.yaml` under `dynamixel_interface/config`)
        name=['joint_1', 'joint_2', 'joint_3', 'joint_4']
    )
    # Funny code
    msg.position = [
        theta_1,
        theta_2,
        theta_3,
        theta_4
    ]
   

    #rospy.loginfo(f'Got desired pose\n[\n\tpos:\n{pose.position}\nrot:\n{pose.orientation}\n]')
    pub.publish(msg)



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
    rospy.init_node('inverse_kinematics', anonymous = True)

    # You spin me right round baby, right round...
    # Just stops Python from exiting and executes callbacks
    rospy.spin()


if __name__ == '__main__':
    main()
