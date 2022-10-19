#! /usr/bin/env python3
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

DROPOFF_1 = mr.RpToTrans(
    np.eye(3), [-0.05, -0.15, 0.03]
)

# TODO: blacklist

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
            self.prev_transform = new_transform
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

        delta_position = self.transform.position - self.prev_transform.position
        delta_time = (self.tf_update_time - self.tf_prev_update_time).to_sec()
        return np.linalg.norm(delta_position) / delta_time
    
    def reconcile(self, other: "Luggage") -> None:
        if other.transform is not None:
            self.transform = other.transform
            self.tf_update_time = other.tf_update_time
        if other.prev_transform is not None:
            self.prev_transform = other.prev_transform
            self.tf_prev_update_time = other.tf_update_time
        if self.color is None and other.color is not None:
            self.color = other.color

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

L1 = 0.053
L2 = 0.117
L3 = 0.095
L4 = 0.113

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

        self.is_busy = False
        self.backup_is_busy = False
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

        self.pose_pub = rospy.Publisher(
            "desired_pose",
            Pose,
            queue_size = 10
        )

        self.joint_pub = rospy.Publisher(
            "desired_joint_states",
            JointState,
            queue_size = 10
        )
        # stores the color of the luggage that is currently being picked up.
        # this should be set to None if we aren't picking up a block.
        self.current_color = None

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
            available_dict[id].update_transform(luggage_tf.transform, time)
    
    def color_callback(self, luggage_color_array: LuggageColorArray):
        if self.is_busy and self.backup_is_busy:
            return

        available_dict = self.backup_luggage_dict if self.is_busy else self.luggage_dict
        for luggage_color in luggage_color_array:
            id = luggage_color.fiducial_id
            available_dict[id].update_color(luggage_color.color_code)
    
    def reconcile(self):
        self.backup_is_busy = True
        self.is_busy = True
        for id, lug in self.backup_luggage_dict.items():
            if id in self.luggage_dict:
                self.luggage_dict[id].reconcile(lug)
            else:
                self.luggage_dict[id] = lug
        self.backup_luggage_dict.clear()
        self.is_busy = False
        self.backup_is_busy = False

    def state_1(self):
        rospy.loginfo("Moving to home configuration...")
        
        msg = JointState(
            # Set header with current time
            header=Header(stamp=rospy.Time.now()),
            # Specify joint names (see `controller_config.yaml` under `dynamixel_interface/config`)
            name=['joint_1', 'joint_2', 'joint_3', 'joint_4'],
            position=[0,0,0,0]
        )
        
        self.joint_pub.publish(msg)

        self._state = 2

    def state_2(self):
        rospy.loginfo("Waiting for conveyor to stop...")

        VELOCITY_THRESHOLD = 0.1

        # Detect Conveyor Movement - Wait For Stationary
        self.is_busy = True
        self.backup_is_busy = False
        self.transform_update = False
        while not self.transform_update:
            pass
        min = float('inf')
        # while True:
        #     for lug in self.luggage_dict.values():
        #         vel = lug.get_velocity()
        #         print(vel)
        #         if vel < min:
        #             min = vel
        #     if min <= VELOCITY_THRESHOLD:
        #         break
        print("uhhh")
        self.reconcile()
        self._state = 3

    def state_3(self):
        rospy.loginfo("Calculating closest luggage...")

        #We set a wild initial minimum distance
        min_id = None
        min_dist = float('inf')
        pose_goto = None
        
        # acquire locks
        self.is_busy = True
        self.backup_is_busy = False

        rospy.loginfo(str(self.luggage_dict))

        #Iterate through each luggage
        for id, lug in self.luggage_dict.items():
            point = lug.transform.position
            point = np.array([point.x, point.y, point.z])
            # distance to base origin
            dist = np.linalg.norm(point)
            if dist < min_dist:
                #Set a new minimum distance to beat!
                min_dist = dist
                #Save the id
                min_id = id
                #Set the transform that we wish to publish to desired_pose
                pose_goto = lug.transform
                self.current_color = lug.color

        self.reconcile()

        rospy.loginfo(f"Closest luggage id={id}.")
        rospy.loginfo("Opening gripper...")
        self.gripper_pub.publish(False)

        rospy.loginfo("Moving to luggage...")

        #Pops the Transform associated with the id of the closest block (i.e the one about to be picked up)
        del self.luggage_dict[min_id]

        #Create message and publish to desired pose
        self.pose_pub.publish(pose_goto)

        # rospy.loginfo("Waiting for joints to reach position...")
        rospy.sleep(3)

        rospy.loginfo("Closing gripper...")
        self.gripper_pub.publish(True)

        self._state = 4

    def state_4(self):
        rospy.loginfo("Moving to drop-off zone...")

        # something with self.current_color
        dropoff_pose = transform_to_pose(SE3_to_transform(DROPOFF_1))
        self.pose_pub.publish(dropoff_pose)

        rospy.loginfo("Waiting for joints to reach position...")
        rospy.sleep(3)

        rospy.loginfo("Opening gripper...")
        self.gripper_pub.publish(True)
        
        self._state = 2

    def main_loop(self):

        # Move To State 1
        self._state = 1

        state = {
            1: self.state_1,
            2: self.state_2,
            3: self.state_3,
            4: self.state_4,
        }

        while not rospy.is_shutdown():
            exec_state = state.get(self._state, None)
            if exec_state is None:
                rospy.logfatal("State Does Not Exist")
                break

            rospy.loginfo("==================================================")
            rospy.loginfo(f"State {self._state}")
            rospy.loginfo("--------------------------------------------------")
            exec_state()
            rospy.loginfo("__________________________________________________")

def main():
    rospy.init_node("robot_planner")
    node = Planner()
    node.main_loop()
    # rospy.spin()

if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass

