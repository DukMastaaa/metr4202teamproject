import rospy
from geometry_msgs.msg import Pose,Point
from robot_msgs.msg import LuggageTransformArray, LuggageTransform, LuggageColorArray, LuggageColor

import modern_robotics as mr
import numpy as np

import constants

class Luggage:
    def __init__(self, transform, color):
        self.transform = transform
        self.color = color

        self.prev_transform = np.eye(4)  # uhh
    
    def update_transform(self, new_transform):
        self.prev_transform = self.transform
        self.transform = new_transform
    
    def update_color(self, new_color):
        # hmmmmm
        print("why is the colour changing...??????")
        self.color = new_color

    def get_velocity(self):
        # calculates velocity based on transform and prev_transform

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
        self.luggage_dict = {}

        lug = Luggage(np.eye(4), 2)
        lug.get_velocity()

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

        self.gripper_pub = rospy.Publisher(
            "desired_gripper_pos",
            Bool,
            queue_size = 10
        )
        self._stage = 0
    
    def transform_callback(self, luggage_tf_array: LuggageTransformArray):
        for luggage_tf in luggage_tf_array.transforms:
            id = luggage_tf.fiducial_id


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
        while all(lug.get_velocity() >= 1 for lug in self.luggage_dict.values()):
            pass

        # Move To State 3
        self._stage = 3

    def state_3(self):
        rospy.loginfo("Detecting luggage position and colours...")

        #We set a wild initial minimum distance
        
        #Iterate through each luggage
        min_id , mind_dist = 1000
        for id, lug in self.luggage_dict.items():
            point = lug.transform.translation
            dist = np.linalg.norm(point)
            if dist < min_dist:
                #Set a new minimum distance to beat!
                min_dist = dist
                #Save the id
                min_id = id
                #Set the transform that we wish to publish to desired_pose
                transform_goto = lug.transform
                lug_colour = lug.color

        #Pops the Transform associated with the id of the closest block (i.e the one about to be picked up)
        del self.luggage_dict[min_id]

        #Create message and publish to desired pose
        msg = Pose(
            Transform = transform_goto
        )

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

