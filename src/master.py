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
