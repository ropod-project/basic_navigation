from __future__ import print_function

import rospy

from utils import Utils
from laser_utils import LaserUtils

class GeometricPlanner(object):

    """Planner for short distance navigation using geometric sensor information"""

    def __init__(self):
        self.laser_utils = LaserUtils()
        rospy.sleep(0.2)

        points = Utils.get_future_poses(1.0, 0.5, 10, 2.0)
        collision_index = self.laser_utils.get_collision_index(points)
        print(collision_index)

    def plan(self, start=(0.0, 0.0), goal=(0.0, 0.0)):
        """
        ASSUMPTION: start is current robot position
        ASSUMPTION: robot is more or less facing goal
        ASSUMPTION: goal is more or less within laser range
        """
        pass
