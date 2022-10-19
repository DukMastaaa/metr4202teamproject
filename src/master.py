import rospy

import modern_robotics as mr
import numpy as np

from typing import Optional
from collections import defaultdict

import constants

class Master:
  """
    Controls sequencing of planner node depending on the level selected.
    Level 1 - one block, rotating conveyor
    Level 2 - multiple blocks, rotating conveyor
    Level 3a - multiple blocks, stationary conveyor
    Level 3b - multiple blocks, rotating conveyor, pickup whilst in motion
    """
    def __init__(self):
        self.pose_pub = rospy.Publisher(
            # Publish to Planner Node?
        )
        
        self._level = 0
    
    def sequence_1_2_3a(self):
      # States 1 - 5, Repeat
      
    def sequence_3b(self):
      
    def main_loop(self):
      level_input = input("Please select a level:\n")
      #self._level = int(level_input)
      
      levels1 = {'1', '2', '3a'}
      levels2 = {'3b'}
      if level_input in levels1:
        sequence_1_2_3a()
      elif level_input in levels2:
        sequence_3b()
      else:
        rospy.logfatal("Level Does Not Exist")

def main():
    node = Master()
    node.main_loop()
    rospy.spin()

if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass
