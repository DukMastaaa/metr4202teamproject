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
        self._level = 0
    
    def level_sequencing(self):
      
    def main_loop(self):
      level_input = input("Please select a level:\n")
      self._level = int(level_input)

def main():
    node = Master()
    node.main_loop()
    rospy.spin()

if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass
