#! /usr/bin/env python3
import copy
import rospy
from geometry_msgs.msg import Pose, Transform, Point, Vector3, Quaternion
from std_msgs.msg import Bool,Header
from robot_msgs.msg import LuggageTransformArray, LuggageTransform, LuggageColorArray, LuggageColor
from sensor_msgs.msg import JointState

import modern_robotics as mr
import numpy as np

from typing import Optional
from collections import defaultdict
from tf_conversions import transformations

from robot_msgs.srv import DoInverseKinematics, DoInverseKinematicsResponse

L1 = 0.053
L2 = 0.117
L3 = 0.095
L4 = 0.113
HOME_POSE = mr.RpToTrans(np.eye(3),[0,0,L1+L2+L3+L4-.05])

SHOW_TO_CAMERA_SE3 = mr.RpToTrans(
    np.eye(3), [15/100, 0, 35/100]
)

# Red zone
DROPOFF_1 = mr.RpToTrans(
    np.eye(3), [-0.05, -0.18, 0.08] # Clockwise turn
)
# Green zone
DROPOFF_2 = mr.RpToTrans(
    np.eye(3),[-0.05,0.18,0.08] # Anti-clockwise turn
)
# Blue zone
DROPOFF_3 = mr.RpToTrans(
    np.eye(3), [-0.15, -0.18, 0.08] # Clockwise turn, increased x
)
# Yellow zone
DROPOFF_4 = mr.RpToTrans(
    np.eye(3),[-0.15,0.18,0.08] # Anti-clockwise turn, increased x
)

def vector_to_point(v: Vector3) -> Point:
    return Point(v.x, v.y, v.x)

def transform_to_pose(tf: Transform) -> Pose:
    return Pose(
        position=vector_to_point(tf.translation),
        orientation=tf.rotation
    )

def SE3_to_transform(T: np.array) -> Transform:
    _, p = mr.TransToRp(T)
    p = Vector3(p[0], p[1], p[2])
    q = transformations.quaternion_from_matrix(T)
    q = Quaternion(x=q[0], y=q[1], z=q[2], w=q[3])
    return Transform(translation=p, rotation=q)

def point_to_np(p: Point) -> np.array:
    return np.array([p.x, p.y, p.z])

# TODO: blacklist
# TODO: link constants as package
# TODO: check for when trajectory finished

class Luggage:
    def __init__(self, transform: Optional[Pose], color: Optional[int]):
        self.transform = transform
        self.prev_transform = transform
        self.color = color
        # saves the time when self.transform was last updated
        self.tf_update_time = None
        self.tf_prev_update_time = None
    
    def update_transform(self, new_transform: Pose, time: rospy.Time) -> None:
        if self.transform is None:
            # initialise to the given transform
            self.prev_transform = None
        else:
            # shift over
            self.prev_transform = self.transform
        self.transform = new_transform
        # update times
        self.tf_prev_update_time = self.tf_update_time
        self.tf_update_time = time
    
    def update_color(self, new_color: int) -> None:
        if self.color is None:
            self.color = new_color
        else:
            # log a warning and don't change the colour
            rospy.logwarn(f"color change attempt from {self.color} to {new_color}")

    def get_velocity(self) -> float:
        # calculates velocity based on transform and prev_transform
        if self.transform is None or self.tf_prev_update_time is None:
            return float('inf')

        delta_position = \
            point_to_np(self.transform.position) - point_to_np(self.prev_transform.position)
        delta_time = (self.tf_update_time - self.tf_prev_update_time).to_sec()
        # print(self.transform, self.prev_transform, self.tf_update_time, self.tf_prev_update_time)
        norm = np.linalg.norm(delta_position)
        if norm == 0.0:
            norm = 1
        else:
            pass

        rospy.loginfo(f"vel={norm}")
        return norm
    
    def reconcile(self, other: "Luggage") -> None:
        if other.transform is not None:
            if other.prev_transform is not None:
                self.prev_transform = other.prev_transform
                self.transform = other.transform
                self.tf_prev_update_time = other.tf_prev_update_time
            else:
                # shift over
                self.prev_transform = self.transform
                self.transform = other.transform
                self.tf_prev_update_time = self.tf_update_time

            self.tf_update_time = other.tf_update_time
        if self.color is None and other.color is not None:
            self.color = other.color

    def __repr__(self):
        return f"Luggage(tf={self.transform},\\prev_tf={self.prev_transform},\\" \
               f"time={self.tf_update_time},\\prev_time={self.tf_prev_update_time})"

class Planner:
    """
    Iterates through fives states to move lugagge to the target drop-off zone.

    State 1: Move Arm To Home Configuration
    State 2: Wait For Conveyor To Stop
    State 3: Record Luggage Positions/Colours, Determine Target Luggage
    State 4: Move Arm To Target Luggage + Pick Up
    State 5: Move Arm To Target Drop-Off Zone. Return To State 1
    """

    def __init__(self):
        # lookup id -> Luggage
        self.luggage_dict = defaultdict(lambda: Luggage(None, None))
        # used by callback when luggage_dict is busy
        self.backup_luggage_dict = defaultdict(lambda: Luggage(None, None))

        # flags to indicate to the callbacks whether the main dictionary
        # or backup dictionary are being used, to prevent generating
        # RuntimeError when dictionaries are modified during iteration
        self.is_busy = False
        self.backup_is_busy = False
        
        # stores ids which are currently blocked from being added to the
        # dictionaries since the grabber is handling them
        self.id_blacklist = []
        
        # stores the id and color code of the luggage that is currently being picked up.
        # this should be set to None if we aren't picking up a block.
        self.current_id = None
        self.current_color = None
        
        # flag set to True when the transform callback updates the luggage dict
        self.transform_update = False

        self.transform_sub = rospy.Subscriber(
            "luggage_transforms",
            LuggageTransformArray,
            self.transform_callback
        )
        self.color_sub = rospy.Subscriber(
            "luggage_colors",
            LuggageColorArray,
            self.color_callback
        )

        self.joint_pub = rospy.Publisher(
            "desired_joint_states",
            JointState,
            queue_size = 10
        )

        self.gripper_pub = rospy.Publisher(
            "desired_gripper_pos",
            Bool,
            queue_size = 10
        )
        self._stage = 0
    
    def transform_callback(self, luggage_tf_array: LuggageTransformArray):
        self.transform_update = True
        time = luggage_tf_array.header.stamp
        if self.is_busy and self.backup_is_busy:
            return
        
        available_dict = self.backup_luggage_dict if self.is_busy else self.luggage_dict
        for luggage_tf in luggage_tf_array.transforms:
            id = luggage_tf.fiducial_id
            if id in self.id_blacklist:
                continue
            available_dict[id].update_transform(luggage_tf.transform, time)
    
    def color_callback(self, luggage_color_array: LuggageColorArray):
        if self.is_busy and self.backup_is_busy:
            return

        available_dict = self.backup_luggage_dict if self.is_busy else self.luggage_dict
        for luggage_color in luggage_color_array:
            id = luggage_color.fiducial_id
            if id in self.id_blacklist:
                continue
            available_dict[id].update_color(luggage_color.color_code)
    
    def reconcile(self):
        # acquire locks
        self.backup_is_busy = True
        self.is_busy = True

        for id, lug in self.backup_luggage_dict.items():
            # rospy.loginfo(str(lug))
            if id in self.luggage_dict:
                self.luggage_dict[id].reconcile(lug)
            else:
                self.luggage_dict[id] = lug
        self.backup_luggage_dict.clear()

        # release locks
        self.is_busy = False
        self.backup_is_busy = False
        
    def desired_theta_e(self, pose: Pose) -> float:
        """Calculates the desired theta_e given a pose."""
        if pose.position.x < 0.2 and pose.position.z < 0.10:
            return np.deg2rad(60)
        elif pose.position.z > 0.13:
            return np.deg2rad(-45)
        else:
            return np.deg2rad(30)
    
    def send_ik(self, pose: Pose, theta_e: float) -> bool:
        """
        Helper function to send the given data to the DoInverseKinematics
        service. Returns whether the IK succeded.
        """
        rospy.wait_for_service("do_inverse_kinematics")
        try:
            do_inverse_kinematics = rospy.ServiceProxy('do_inverse_kinematics', DoInverseKinematics)
            resp = do_inverse_kinematics(pose, theta_e)
            return resp.success.data
        except rospy.ServiceException as e:
            print(f"Service call failed: {e}")
            return False

    def state_1(self):
        rospy.loginfo("Moving to intial configuration:")
        show_to_camera_pose = transform_to_pose(SE3_to_transform(SHOW_TO_CAMERA_SE3))
        self.pose_pub.publish(show_to_camera_pose)
        rospy.sleep(2)

        self.state_num = 2

    def state_2(self):
        #Moving to home_configuration
        rospy.loginfo("Moving to home configuration...")
        
        msg = JointState(
            header=Header(stamp=rospy.Time.now()),
            name=['joint_1', 'joint_2', 'joint_3', 'joint_4'],
            position=[0, 0, 0, 0]
        )
        self.joint_pub.publish(msg)
        rospy.sleep(2)

        rospy.loginfo("Waiting for conveyor to stop and for luggage to be present...")

        VELOCITY_THRESHOLD = 0.0008

        min = float('inf')
        while True:
            # acquire locks
            self.is_busy = True
            self.backup_is_busy = False
            # get minimum
            # print('.', end="")
            # print('You are in while loop')
            if self.transform_update:
                # rospy.loginfo(str(list(self.luggage_dict.values())))
                self.transform_update = False
            # rospy.sleep(0.5)
            if len(self.luggage_dict) > 0 and \
                    all(lug.get_velocity() <= VELOCITY_THRESHOLD
                    for lug in self.luggage_dict.values()):
                break

            # release locks
            self.reconcile()
            
            # rospy.sleep(0.5)
            
        rospy.loginfo("detected!")
        self.state_num = 3

    def state_3(self):
        rospy.loginfo("Calculating closest luggage...")

        # we set a wild initial minimum distance
        min_id = None
        min_dist = float('inf')
        min_pose = None
        
        # acquire locks
        self.is_busy = True
        self.backup_is_busy = False

        # iterate through each luggage
        for id, lug in self.luggage_dict.items():
            point = lug.transform.position
            point = np.array([point.x, point.y, point.z])
            # distance to base origin
            dist = np.linalg.norm(point)
            if dist < min_dist:
                # set a new minimum distance to beat!
                min_dist = dist
                # save the id
                min_id = id
                # set the transform that we wish to publish to desired_pose
                min_pose = lug.transform
                # save colour of block so we know what drop-off zone to go to
                self.current_color = lug.color

        self.current_id = id
        self.id_blacklist.append(self.current_id)
        # release locks
        self.reconcile()

        rospy.loginfo(f"Closest luggage id={id}.")
        rospy.loginfo("Opening gripper...")
        self.gripper_pub.publish(False)

        rospy.loginfo("Moving to above luggage...")

        # Pops the Transform associated with the id of the closest block (i.e the one about to be picked up)
        del self.luggage_dict[min_id]

        above_pose = copy.deepcopy(min_pose)
        above_pose.position.z += 0.03
        theta_e = self.desired_theta_e(above_pose)
        success = self.send_ik(above_pose, theta_e)
        
        if not success:
            self.state_num = 2
            rospy.sleep(1)
            return

        rospy.loginfo("Waiting for joints to reach position...")
        rospy.sleep(3)
        
        rospy.loginfo("Moving down to luggage...")
        theta_e = self.desired_theta_e(min_pose)
        success = self.send_ik(min_pose, theta_e)
        if not success:
            self.state_num = 2
            rospy.sleep(1)
            return

        rospy.loginfo("Waiting for joints to reach position...")
        rospy.sleep(2)

        rospy.loginfo("Closing gripper...")
        self.gripper_pub.publish(True)

        self.state_num = 4

    def state_4(self):
        rospy.loginfo("Moving to show block to camera:")
        show_to_camera_pose = transform_to_pose(SE3_to_transform(SHOW_TO_CAMERA_SE3))
        self.pose_pub.publish(show_to_camera_pose)

        rospy.loginfo("Waiting for joints to reach position...")
        rospy.sleep(3)
        
        self.state_num = 5
    
    def state_5(self):
        rospy.loginfo("Moving to above drop-off zone...")
        # the proper code should involve self.current_color
        # to decide what dropoff zone to use
        
        dropoff_zone = None
        if self.current_color == 0:
            dropoff_zone = DROPOFF_1
            print("Red detected!")
        elif self.current_color == 1:
            dropoff_zone = DROPOFF_2
            print("Green detected!")
        elif self.current_color == 2:
            dropoff_zone = DROPOFF_3
            print("Blue detected!")
        elif self.current_color == 3:
            dropoff_zone = DROPOFF_4
            print("Yellow detected!")
        elif self.current_color == None:
            dropoff_zone = DROPOFF_1
            print("No update to color!") # Callback is not updating self.current_color
        else:
            dropoff_zone = DROPOFF_1
            print("No block detected!") # Pixel color is not any of the main block colors
            
        dropoff_pose = transform_to_pose(SE3_to_transform(dropoff_zone))
        above_dropoff_pose = copy.deepcopy(dropoff_pose)
        above_dropoff_pose.position.z += 0.1

        self.pose_pub.publish(above_dropoff_pose)

        rospy.loginfo("Waiting for joints to reach position...")
        rospy.sleep(3)
        
        rospy.loginfo("Moving to drop-off zone...")
        self.pose_pub.publish(dropoff_pose)

        rospy.loginfo("Waiting for joints to reach position...")
        rospy.sleep(3)

        rospy.loginfo("Opening gripper...")
        self.gripper_pub.publish(False)
        rospy.sleep(0.5)
        self.id_blacklist.remove(self.current_id)
       
        self.state_num = 2

    def main_loop(self):
        self.state_num = 1
        state_num_to_function = {
            1: self.state_1,
            2: self.state_2,
            3: self.state_3,
            4: self.state_4,
            5: self.state_5,
        }

        while not rospy.is_shutdown():
            exec_state = state_num_to_function.get(self.state_num, None)
            if exec_state is None:
                rospy.logfatal("State Does Not Exist")
                break

            rospy.loginfo("==================================================")
            rospy.loginfo(f"State {self.state_num}")
            rospy.loginfo("--------------------------------------------------")
            exec_state()
            rospy.loginfo("__________________________________________________")

def main():
    rospy.init_node("robot_planner")
    node = Planner()
    node.main_loop()

if __name__ == '__main__':
    main()
