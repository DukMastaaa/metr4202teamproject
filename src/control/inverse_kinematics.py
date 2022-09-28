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
 
# maps between theta_i to the actual IDs of the dynamixels
# in the arm 
JOINT_MAPPING = {
    1: 2,
    2: 3,
    3: 4,
    4: 1
}

def solution_2r(L1, L2, px, py):
    c2 = (px**2 + py**2 - L1**2 - L2**2) / (2 * L1 * L2)
    theta2 = atan2(+sqrt(1 - c2**2), c2)
    theta1 = atan2(py, px) - atan2(L2 * sin(theta2), L1 + L2 * cos(theta2))
    return theta1, theta2

def inverse_kinematics(pose: Pose) -> JointState:
    global pub
    L1 = 0.053
    L2 = 0.12
    L3 = 0.095
    L4 = 0.113
    px = pose.position.x
    py = pose.position.y
    pz = pose.position.z

    # px = (L2 + L3 + L4) / 2
    # py = 0
    # pz = L1 * 0.5
    # print(px, py, pz)
    theta_e = np.deg2rad(70)

    # get position of joint 4 (br*cos(phi), br*sin(phi))
    # and project onto planar (r, z) coordinates
    br = sqrt(px**2 +py**2)- L4*cos(theta_e)
    phi = atan2(py,px)
    # phi = 0
    bz = pz + L4*sin(theta_e) - L1

    print(f"({br}, {bz})")

    # c_b = (br**2 + (bz-L1)**2 - L2**2 - L3**2) / (2 * L2 * L3)
    c_b = (br**2 + bz**2 - L2**2 - L3**2) / (2 * L2 * L3)
    if abs(c_b) >= 1:
        raise ValueError("c_b >= 1")
    print(c_b)
    # we pick negative branch
    theta_b = atan2(+sqrt(1 - c_b**2), c_b)
    theta_a = atan2(bz, br) - atan2(L3 * sin(-theta_b), L2 + L3 * c_b)
    print(f"b: {np.rad2deg(theta_b)}, a: {np.rad2deg(theta_a)}")

    # theta_1 = np.deg2rad(90) - phi
    theta_1 = phi
    # convert from lecture formulas to our actual theta_2, theta_3
    theta_2 = np.deg2rad(90) - theta_a
    # theta_2 = -theta_2
    theta_3 = theta_b

    theta_4 = theta_e - theta_2 - theta_3 + np.deg2rad(90)

    print("before offset:", np.rad2deg(theta_1), np.rad2deg(theta_2), np.rad2deg(theta_3), np.rad2deg(theta_4))

    theta_2 = np.deg2rad(90) - theta_2
    theta_3 = - theta_3
    theta_4 = theta_4
    print(theta_4)
    # theta_4 = np.deg2rad(90)
    # theta_4 += np.deg2rad(10)

    print("after offset:", np.rad2deg(theta_1), np.rad2deg(theta_2), np.rad2deg(theta_3), np.rad2deg(theta_4))
    
    # Create message of type JointState
    msg = JointState(
        # Set header with current time
        header=Header(stamp=rospy.Time.now()),
        # Specify joint names (see `controller_config.yaml` under `dynamixel_interface/config`)
        name=['joint_1', 'joint_2', 'joint_3', 'joint_4']
        # name=['joint_2', 'joint_3', 'joint_4', 'joint_1']
    )
    # Funny code
    msg.position = [
        theta_4,
        theta_1,
        theta_2,
        theta_3,
    ]
    # msg.position = [
    #     theta_1,
    #     theta_2,
    #     theta_3,
    #     theta_4
    # ]

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
