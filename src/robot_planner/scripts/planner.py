import rospy
from geometry_msgs.msg import Pose, Transform, Point
from robot_msgs.msg import LuggageTransformArray, LuggageTransform, LuggageColorArray, LuggageColor

import modern_robotics as mr
import numpy as np

from typing import Optional
from collections import defaultdict

import constants

class Luggage:
    def __init__(self, transform: Optional[Transform], color: Optional[int]):
        self.transform = transform
        self.prev_transform = transform
        self.color = color
        # saves the time when self.transform was last updated
        self.tf_update_time = None
        self.tf_prev_update_time = None
    
    def update_transform(self, new_transform: Transform, time: rospy.Time) -> None:
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

    def get_velocity(self) -> Optional[float]:
        # calculates velocity based on transform and prev_transform
        if self.transform is None:
            return None
        delta_position = self.transform.translation - self.prev_transform.translation
        delta_time = (self.tf_update_time - self.tf_prev_update_time).to_sec()
        return np.linalg.norm(delta_position) / delta_time

def transform_to_pose(transform: Transform) -> Pose:
    translation = transform.translation
    return Pose(
        position=Point(translation.x, translation.y, translation.z),
        orientation=transform.rotation
    )

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
        time = luggage_tf_array.header.stamp
        for luggage_tf in luggage_tf_array.transforms:
            id = luggage_tf.fiducial_id
            self.luggage_dict[id].update_transform(luggage_tf.transform, time)
    
    def color_callback(self, luggage_color_array: LuggageColorArray):
        for luggage_color in luggage_color_array:
            id = luggage_color.fiducial_id
            self.luggage_dict[id].update_color(luggage_color.color_code)

    def state_1(self):
        rospy.loginfo("Moving to home configuration...")
        
        msg = Pose(
                position = Point(0, 0, constants.L1 + constants.L2 + constants.L3 + constants.L4)
            )
        self.pose_pub.publish(msg)

        self._state = 2

    def state_2(self):
        rospy.loginfo("Waiting for conveyor to stop...")

        # Detect Conveyor Movement - Wait For Stationary
        while all(lug.get_velocity() >= 1 for lug in self.luggage_dict.values()):
            pass

        self._stage = 3

    def state_3(self):
        rospy.loginfo("Calculating closest luggage...")

        #We set a wild initial minimum distance
        min_id = None
        min_dist = float('inf')
        transform_goto = None
        
        #Iterate through each luggage
        for id, lug in self.luggage_dict.items():
            point = lug.transform.translation
            # distance to base origin
            dist = np.linalg.norm(point)
            if dist < min_dist:
                #Set a new minimum distance to beat!
                min_dist = dist
                #Save the id
                min_id = id
                #Set the transform that we wish to publish to desired_pose
                transform_goto = lug.transform
                self.current_color = lug.color

        rospy.loginfo(f"Closest luggage id={id}.")
        rospy.loginfo("Moving to luggage:")

        #Pops the Transform associated with the id of the closest block (i.e the one about to be picked up)
        del self.luggage_dict[min_id]

        #Create message and publish to desired pose
        msg = transform_to_pose(transform_goto)
        self.pose_pub.publish(msg)

        self._stage = 4

    def state_4(self):
        rospy.loginfo("Moving to drop-off zone...")
        #We must pick up the block: 
        self.gripper_pub.publish(1)

        # Set Drop-Off Zone Pose + Publish To IK
        dropoff_zone = None

        if dropoff_zone is None:
            rospy.logfatal("No colours match drop zones")
            rospy.signal_shutdown("Colour not found")
            return

        x = 0
        y = 0
        z = 0

        msg = Pose(
                position = Point(x, y, z)
            )
        self.pose_pub.publish(msg)

        rospy.loginfo("Releasing Luggage...")

        # Move To State 2
        self._state = 2

    def main_loop(self):

        # Move To State 1
        self._state = 1

        state = {
            1: self.state_1,
            2: self.state_2,
            3: self.state_3,
            4: self.state_4,
            4: self.state_5
        }

        while not rospy.is_shutdown():
            exec_state = state.get(self._state, None)
            if exec_state is None:
                rospy.logfatal("State Does Not Exist")
                break

            rospy.loginfo("==================================================")
            rospy.loginfo(f"Stage {self._state}")
            rospy.loginfo("--------------------------------------------------")
            exec_state()
            rospy.loginfo("__________________________________________________")

def main():
    node = Planner()
    node.main_loop()

    rospy.spin()

if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass

