from __future__ import print_function

import math
import rospy
import sensor_msgs.point_cloud2 as pc2
from laser_geometry import LaserProjection

from geometry_msgs.msg import Point, PolygonStamped
from sensor_msgs.msg import LaserScan, PointCloud2
from utils import Utils

class LaserUtils(object):

    """Utils class to use laser data with basic navigation"""

    def __init__(self, **kwargs):
        self.robot_frame = rospy.get_param('~robot_frame', 'load/base_link')

        # class variables
        self.laser_proj = LaserProjection()
        self.footprint = [[-0.33, 0.33], [0.33, 0.33], [0.33, -0.33], [-0.33, -0.33]] # TODO use rosparam
        self.base_link_to_laser_offset = (0.2, 0.0) # TODO use tf?
        footprint_padding = kwargs.get('footprint_padding', 0.1)
        self.set_footprint_padding(footprint_padding)
        self.cloud = None
        self.debug = kwargs.get('debug', False)

        # publisher
        self.pc_pub = rospy.Publisher('~pc_debug', PointCloud2, queue_size=1)
        self.footprint_pub = rospy.Publisher('~footprint_debug', PolygonStamped, queue_size=1)

        # subscribers
        laser_sub = rospy.Subscriber('~laser', LaserScan, self.laser_cb)

        rospy.sleep(0.5)
        if self.debug:
            self.pub_debug_footprint()
            rospy.sleep(0.2)

    def laser_cb(self, msg):
        self.cloud = self.laser_proj.projectLaser(msg, channel_options=LaserProjection.ChannelOption.NONE)
        if self.debug:
            self.pc_pub.publish(self.cloud)

    def is_safe_from_colliding_at(self, x=0.0, y=0.0, theta=0.0):
        footprint = self.get_footprint_at(x, y, theta)
        if self.debug:
            self.pub_debug_footprint(footprint)
        points = pc2.read_points(self.cloud, skip_nans=True, field_names=("x", "y"))
        for p in points:
            if self.is_inside_polygon(p[0], p[1], footprint):
                return False
        return True

    def get_footprint_at(self, x=0.0, y=0.0, theta=0.0):
        new_footprint = []
        for p in self.padded_footprint:
            rotated_point = Utils.get_rotated_point((p.x, p.y), theta)
            new_footprint.append(Point(x=rotated_point[0]+x, y=rotated_point[1]+y))
        return new_footprint

    def get_collision_index(self, points):
        for i, p in enumerate(points):
            safe = self.is_safe_from_colliding_at(*p)
            if not safe:
                return i
        return len(points)

    def pub_debug_footprint(self, footprint=None):
        if footprint is None:
            footprint = self.padded_footprint
        polygon_msg = PolygonStamped()
        polygon_msg.header.stamp = rospy.Time.now()
        polygon_msg.header.frame_id = self.robot_frame
        polygon_msg.polygon.points = footprint
        self.footprint_pub.publish(polygon_msg)

    def set_footprint_padding(self, padding):
        if padding < 0.0:
            return
        self.padding = padding
        self.initialise_padded_footprint()

    def initialise_padded_footprint(self):
        self.padded_footprint = []
        for p in self.footprint:
            sign_x = 1.0 if p[0] > 0 else -1.0
            sign_y = 1.0 if p[1] > 0 else -1.0
            x = p[0] + (sign_x * self.padding)
            y = p[1] + (sign_y * self.padding)
            self.padded_footprint.append(Point(x=x, y=y))

    def is_inside_polygon(self, x, y, points):
        """checks if point (x,y) is inside the polygon created by points

        :x: int/float
        :y: int/float
        :points: list of geometry_msgs.Point
        :returns: bool

        """
        xPoints = [p.x for p in points]
        yPoints = [p.y for p in points]

        return Utils.ray_tracing_algorithm(xPoints, yPoints,
                                           x+self.base_link_to_laser_offset[0],
                                           y+self.base_link_to_laser_offset[1])

    def get_recovery_direction(self):
        points = pc2.read_points(self.cloud, skip_nans=True, field_names=("x", "y"))
        closest_point = min(points, key=lambda point: Utils.get_distance(point[0], point[1]))
        return math.atan2(closest_point[1], closest_point[0])
