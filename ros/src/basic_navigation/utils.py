from __future__ import print_function

import tf
import math
import rospy
from nav_msgs.msg import Path
from geometry_msgs.msg import PoseStamped, Pose, TransformStamped, Quaternion
from geometry_msgs.msg import Twist, Point

class Utils(object):

    """Utility functions used for basic 2d navigation"""

    @staticmethod
    def get_pose_from_x_y_theta(x, y, theta):
        """Return a Pose object from x, y and theta
        :x: float
        :y: float
        :theta: float
        :returns: geometry_msgs.Pose

        """
        pose = Pose()
        pose.position.x = x
        pose.position.y = y

        quat = tf.transformations.quaternion_from_euler(0.0, 0.0, theta)
        pose.orientation = Quaternion(*quat)
        return pose

    @staticmethod
    def get_pose_stamped_from_frame_x_y_theta(frame, x, y, theta):
        """Return a Pose object from x, y and theta
        :x: float
        :y: float
        :theta: float
        :frame: string
        :returns: geometry_msgs.PoseStamped

        """
        pose = PoseStamped()
        pose.pose = Utils.get_pose_from_x_y_theta(x, y, theta)
        pose.header.stamp = rospy.Time.now()
        pose.header.frame_id = frame
        return pose

    @staticmethod
    def get_x_y_theta_from_pose(pose):
        """Return a tuple(x, y, theta) from Pose objects

        :pose: geometry_msgs/Pose
        :returns: tuple(x, y, theta)

        """
        quat = (pose.orientation.x, pose.orientation.y, pose.orientation.z, pose.orientation.w)
        theta = tf.transformations.euler_from_quaternion(quat)[2]
        return (pose.position.x, pose.position.y, theta)

    @staticmethod
    def get_static_transform_from_x_y_theta(x, y, theta, frame_id="odom"):
        """Create a TransformedStamped message from x y and theta to 0, 0 and 0

        :x: float
        :y: float
        :theta: float
        :returns: geometry_msgs.TransformStamped
        """
        transform = TransformStamped()

        transform.header.stamp = rospy.Time.now()
        transform.header.frame_id = frame_id
        transform.child_frame_id = "start_pose"

        transform.transform.translation.x = x
        transform.transform.translation.y = y
        transform.transform.translation.z = 0.0

        quat = tf.transformations.quaternion_from_euler(0.0, 0.0, theta)
        transform.transform.rotation = Quaternion(*quat)
        return transform

    @staticmethod
    def get_shortest_angle(angle1, angle2):
        """Compute the angular distance between two angles (in radians)

        :angle1: float
        :angle2: float
        :returns: float
        """
        return math.atan2(math.sin(angle1 - angle2), math.cos(angle1 - angle2))

    @staticmethod
    def get_reverse_angle(angle):
        """Compute the angle facing opposite of given angle and ensures that the 
        returned angle is between pi and -pi
        ASSUMPTION: angle is always between pi and -pi

        :angle: float
        :returns: float
        """
        reverse_angle = angle - math.pi
        if reverse_angle < -math.pi:
            reverse_angle = reverse_angle + (2 * math.pi)
        return reverse_angle

    @staticmethod
    def get_distance(delta_x, delta_y):
        """Compute cartesian distance given individual distance in x and y axis

        :delta_x: float
        :delta_y: float
        :returns: float

        """
        return (delta_x**2 + delta_y**2)**0.5

    @staticmethod
    def get_distance_between_points(p1, p2):
        """Compute cartesian distance given two points

        :p1: tuple(float, float)
        :p2: tuple(float, float)
        :returns: float

        """
        return Utils.get_distance(p1[0]-p2[0], p1[1]-p2[1])

    @staticmethod
    def clip(value, max_allowed=1.0, min_allowed=0.1):
        """Clip the provided value to be between the given range

        :value: float
        :max_allowed: float
        :min_allowed: float
        :returns: float

        """
        sign = 1.0 if value > 0.0 else -1.0
        return sign * min(max_allowed, max(min_allowed, abs(value)))

    @staticmethod
    def get_twist(x=0.0, y=0.0, theta=0.0):
        """Return twist ros message object.

        :x: float
        :y: float
        :theta: float
        :returns: geometry_msgs.msg.Twist

        """
        msg = Twist()
        msg.linear.x = x
        msg.linear.y = y
        msg.angular.z = theta
        return msg

    @staticmethod
    def get_future_positions(vel_x, vel_theta, num_of_points, future_time):
        """
        Calculate a bunch of points where the robot would be (in base_link) in future when 
        certain velocity are executed.
        The last point would be the position of robot almost at `future_time` and first
        point is the robot's current position.

        :vel_x: float (forward linear velocity)
        :vel_theta: float (angular yaw velocity)
        :num_of_points: int (number of points to generate)
        :future_time: float (seconds)
        :returns: list of geometry_msgs.Point

        """
        dist = abs(vel_x) * future_time
        angular_dist = abs(vel_theta) * future_time
        radius = dist/angular_dist

        sign_x = 1 if vel_x > 0 else -1
        sign_theta = 1 if vel_theta > 0 else -1

        theta_inc = angular_dist/num_of_points
        points = []
        for i in range(num_of_points):
            theta = i * theta_inc
            x = sign_x * (radius * math.sin(theta))
            y = sign_theta * radius * (1 - math.cos(theta))
            points.append(Point(x=x, y=y, z=0.0))
        return points

    @staticmethod
    def get_path_msg_from_points(points, frame_id):
        path_msg = Path()
        path_msg.header.stamp = rospy.Time.now()
        path_msg.header.frame_id = frame_id

        for p in points:
            pose = PoseStamped()
            pose.pose.position = p
            pose.pose.orientation.w = 1.0
            path_msg.poses.append(pose)
        return path_msg

