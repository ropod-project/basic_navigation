from __future__ import print_function

import rospy

from sensor_msgs.msg import LaserScan
from utils import Utils

class LaserUtils(object):

    """Utils class to use laser data with basic navigation"""

    def __init__(self, **kwargs):
        self.min_laser_dist_front = 0.0
        self.min_laser_dist_back = 0.0
        self.front_laser_ranges = None
        self.back_laser_ranges = None
        self.moving_backward = False

        # lasers
        laser_fov = rospy.get_param('~laser_fov', 1.0)
        self.half_laser_fov = laser_fov/2
        # self.min_angle_front = rospy.get_param('~min_angle_front', -0.5)
        # self.max_angle_front = rospy.get_param('~max_angle_front', 0.5)
        # self.min_angle_back = rospy.get_param('~min_angle_back', -2.9)
        # self.max_angle_back = rospy.get_param('~max_angle_back', 2.9)
        self.base_link_laser_dist = rospy.get_param('~base_link_laser_dist', 0.0)

        self.safety_dist = rospy.get_param('~safety_dist', 0.4)

        self._collision_laser_pub = rospy.Publisher('~collision_scan', LaserScan, queue_size=1)
        laser_sub = rospy.Subscriber('~laser', LaserScan, self.laser_cb)

    def laser_cb(self, msg):
        angle_inc = msg.angle_increment

        if msg.angle_min > -self.half_laser_fov or msg.angle_max < self.half_laser_fov:
            rospy.logerr("laser message data not available in filter range")

        first_index = int((-self.half_laser_fov - msg.angle_min)/angle_inc)
        last_index = int((self.half_laser_fov - msg.angle_min )/angle_inc)
        self.front_laser_ranges = msg.ranges[first_index:last_index]
        self.min_laser_dist_front = min(self.front_laser_ranges)

        filtered_ranges_back = []
        if msg.angle_min > -3.1 or msg.angle_max < 3.1:
            rospy.logerr("laser message data not available in filter range")

        first_index = int(self.half_laser_fov/angle_inc)
        last_index = int((msg.angle_max - self.half_laser_fov - msg.angle_min )/angle_inc)
        last_index -= len(msg.ranges)
        self.back_laser_ranges = [msg.ranges[i] for i in range(first_index, last_index, -1)]
        self.min_laser_dist_back = min(self.back_laser_ranges)

        filtered_laser_msg = LaserScan()
        filtered_laser_msg.header = msg.header
        filtered_laser_msg.angle_min = -self.half_laser_fov
        filtered_laser_msg.angle_max = self.half_laser_fov
        filtered_laser_msg.angle_increment = msg.angle_increment
        filtered_laser_msg.range_min = msg.range_min
        filtered_laser_msg.range_max = msg.range_max
        filtered_laser_msg.ranges = self.front_laser_ranges
        self._collision_laser_pub.publish(filtered_laser_msg)

    def get_laser_dist_at(self, angle):
        if angle < -self.half_laser_fov or angle > self.half_laser_fov:
            return float('inf')
        angle_inc = (2*self.half_laser_fov)/len(self.front_laser_ranges)
        range_index = int(round((angle+self.half_laser_fov)/angle_inc))
        return self.front_laser_ranges[range_index]

    def get_collision_index(self, points):
        if self.moving_backward:
            obs_dist_from_base = self.min_laser_dist_back - self.base_link_laser_dist - self.safety_dist
        else:
            obs_dist_from_base = self.min_laser_dist_front + self.base_link_laser_dist - self.safety_dist

        for i in range(len(points)):
            if points[i].x > obs_dist_from_base:
                return i
        return len(points)

