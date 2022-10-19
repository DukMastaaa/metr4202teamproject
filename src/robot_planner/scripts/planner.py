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
    
    def update_transform(self, new_transform: Transform) -> None:
        if self.transform is None:
            # initialise to the given transform
            self.prev_transform = new_transform
        else:
            # shift over
            self.prev_transform = self.transform
        self.transform = new_transform
    
    def update_color(self, new_color: int) -> None:
        if self.color is None:
            self.color = new_color
        else:
            # log a warning and don't change the colour
            rospy.loginfo(f"color change attempt from {self.color} to {new_color}")

    def get_velocity(self) -> Optional[float]:
        # calculates velocity based on transform and prev_transform
        if self.transform is None:
            return None
        
        pass

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
        self._stage = 0
    
    def transform_callback(self, luggage_tf_array: LuggageTransformArray):
        for luggage_tf in luggage_tf_array.transforms:
            id = luggage_tf.fiducial_id
            self.luggage_dict[id].update_transform(luggage_tf.transform)
    
    def color_callback(self, luggage_color_array: LuggageColorArray):
        for luggage_color in luggage_color_array:
            id = luggage_color.fiducial_id
            self.luggage_dict[id].update_color(luggage_color.color_code)

    def wait_conveyor(self):
        return True

    def state_1(self):
        rospy.loginfo("Moving to home configuration...")
        
        # Set Home Configuration Pose + Publish To IK
        msg = Pose(
                position = Point(0, 0, constants.L1 + constants.L2 + constants.L3 + constants.L4)
            )
        self.pose_pub.publish(msg)

        # Move To State 2
        self._state = 2

    def state_2(self):
        rospy.loginfo("Waiting for conveyor to stop...")

        # Detect Conveyor Movement - Wait For Stationary
        self.wait_conveyor()

        # Move To State 3
        self._stage = 3

    def state_3(self):
        rospy.loginfo("Detecting luggage position and colours...")

        # Detect Luggage + Colours, Determine Closest Block

        # Move To State 4

        self._stage = 4

    def state_4(self):
        rospy.loginfo("Moving arm to target luggage...")

        # Set Luggage Position Pose + Publish To IK
        x = 0
        y = 0
        z = 0

        msg = Pose(
                position = Point(x, y, z)
            )
        self.pose_pub.publish(msg)

        rospy.loginfo("Grabbing Luggage...")

        # Move To State 5
        self._state = 5
        
    def state_5(self):
        rospy.loginfo("Moving to drop-off zone...")

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

