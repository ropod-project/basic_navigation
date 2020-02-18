from __future__ import print_function

import tf
import copy
import math
import rospy
from std_msgs.msg import Empty
from nav_msgs.msg import Path, Odometry
from geometry_msgs.msg import PoseStamped, PointStamped
from geometry_msgs.msg import Twist, PoseWithCovarianceStamped
from sensor_msgs.msg import LaserScan
from maneuver_navigation.msg import Feedback as ManeuverNavFeedback

from utils import Utils
from global_planner_utils import GlobalPlannerUtils

class BasicNavigation(object):

    """Navigation to move in a straight line towards a goal"""

    def __init__(self):
        # Class variables
        self.goal = None
        self.curr_pos = None
        self.plan = None
        self.reached_goal_once = False
        self.moving_backward = False
        self.global_frame = rospy.get_param('~global_frame', 'map')
        self.robot_frame = rospy.get_param('~robot_frame', 'load/base_link')
        self.use_global_planner = rospy.get_param('~use_global_planner', False)
        self.allow_backward_motion = rospy.get_param('~allow_backward_motion', False)
        self.num_of_retries = rospy.get_param('~num_of_retries', 3)
        self.retry_attempts = 0
        self.current_vel = 0.0
        self.min_laser_dist_front = 0.0
        self.min_laser_dist_back = 0.0

        # lasers
        self.min_angle_front = rospy.get_param('~min_angle_front', -0.5)
        self.max_angle_front = rospy.get_param('~max_angle_front', -0.5)
        self.min_angle_back = rospy.get_param('~min_angle_back', -0.5)
        self.max_angle_back = rospy.get_param('~max_angle_back', 0.5)
        self.base_link_laser_dist = rospy.get_param('~base_link_laser_dist', 0.0)

        # tolerances
        self.heading_tolerance = rospy.get_param('~heading_tolerance', 0.5)
        self.goal_dist_tolerance = rospy.get_param('~goal_dist_tolerance', 0.1)
        self.goal_theta_tolerance = rospy.get_param('~goal_theta_tolerance', 0.1)
        self.latch_xy_goal = rospy.get_param('~latch_xy_goal', True)
        self.waypoint_goal_tolerance = rospy.get_param('~waypoint_goal_tolerance', 0.3)
        self.goal_path_start_point_tolerance = rospy.get_param('~goal_path_start_point_tolerance', 1.0)
        max_safe_costmap_val = rospy.get_param('~max_safe_costmap_val', 80)

        # controller params
        self.p_theta_in_place = rospy.get_param('p_theta_in_place', 5.0)
        self.p_theta = rospy.get_param('p_theta', 1.0)
        self.p_linear = rospy.get_param('p_linear', 1.0)
        self.max_theta_vel = rospy.get_param('~max_theta_vel', 0.5)
        self.min_theta_vel = rospy.get_param('~min_theta_vel', 0.005)
        self.max_linear_vel = rospy.get_param('~max_linear_vel', 0.3)
        self.min_linear_vel = rospy.get_param('~min_linear_vel', 0.1)
        max_linear_acc_per_second = rospy.get_param('~max_linear_acc', 0.5)
        control_rate = rospy.get_param('~control_rate', 5.0)
        self.max_linear_acc = max_linear_acc_per_second/control_rate
        self.safety_dist = rospy.get_param('~safety_dist', 0.4)
        self.future_pos_lookahead_time = rospy.get_param('~future_pos_lookahead_time', 3.0)

        # recovery
        self.recovery_enabled = rospy.get_param('~recovery_enabled', False)
        if self.recovery_enabled:
            self.recovery_wait_duration = rospy.get_param('~recovery_wait_duration', 1.0)
            self.recovery_motion_duration = rospy.get_param('~recovery_motion_duration', 2.0)
            self.recovery_vel = rospy.get_param('~recovery_vel', -0.1)

        self.costmap_to_vel_multiplier = (self.max_linear_vel-self.min_linear_vel)/max_safe_costmap_val

        self._tf_listener = tf.TransformListener()

        # Publishers
        self._cmd_vel_pub = rospy.Publisher('~cmd_vel', Twist, queue_size=1)
        self._path_pub = rospy.Publisher('~path', Path, queue_size=1)
        self._traj_pub = rospy.Publisher('~trajectory', Path, queue_size=1)
        self._laser_pub = rospy.Publisher('~collision_scan', LaserScan, queue_size=1)
        self._collision_lookahead_point_pub = rospy.Publisher('~collision_point',
                                                              PointStamped,
                                                              queue_size=1)
        self._nav_feedback_pub = rospy.Publisher('~nav_feedback',
                                                 ManeuverNavFeedback,
                                                 queue_size=1)

        # Subscribers
        laser_sub = rospy.Subscriber('~laser', LaserScan, self.laser_cb)
        goal_sub = rospy.Subscriber('~goal', PoseStamped, self.goal_cb)
        path_sub = rospy.Subscriber('~goal_path', Path, self.path_cb)
        cancel_goal_sub = rospy.Subscriber('~cancel', Empty, self.cancel_current_goal)
        odom_sub = rospy.Subscriber('~odom', Odometry, self.odom_cb)

        # Global planner
        if self.use_global_planner:
            self.global_planner_utils = GlobalPlannerUtils()

        rospy.sleep(0.5)
        rospy.loginfo('Initialised')

    def run_once(self):
        """
        Main event loop
        """
        self.get_current_position_from_tf()

        if self.goal is None:
            return

        if self.curr_pos is None:
            rospy.logwarn('Current pose is not available')
            return

        if self.plan is None:
            if self.use_global_planner:
                rospy.loginfo('Trying to get global plan')
                self._get_global_plan()
                return
            else:
                rospy.loginfo('Getting straight line path')
                self._get_straight_line_plan()
                return

        curr_goal = Utils.get_x_y_theta_from_pose(self.plan[0].pose)
        dist = Utils.get_distance_between_points(self.curr_pos[:2], curr_goal[:2])
        if len(self.plan) == 1 and (dist < self.goal_dist_tolerance or (self.latch_xy_goal and self.reached_goal_once)) :
            self.reached_goal_once = True
            angular_dist = Utils.get_shortest_angle(curr_goal[2], self.curr_pos[2])
            if abs(angular_dist) < self.goal_theta_tolerance:
                rospy.loginfo('REACHED GOAL')
                self.publish_nav_feedback(ManeuverNavFeedback.SUCCESS)
                self._reset_state()
                return
            else:
                self._rotate_in_place(theta_error=angular_dist)
                return
        if dist < self.waypoint_goal_tolerance and len(self.plan) > 1:
            rospy.loginfo('Reached waypoint')
            old_wp = self.plan.pop(0)
            self.publish_nav_feedback(ManeuverNavFeedback.BUSY)

        heading = math.atan2(curr_goal[1]-self.curr_pos[1], curr_goal[0]-self.curr_pos[0])
        heading_diff = Utils.get_shortest_angle(heading, self.curr_pos[2])
        if self.allow_backward_motion:
            heading_diff_backward = Utils.get_shortest_angle(heading,
                                                             Utils.get_reverse_angle(self.curr_pos[2]))
            if abs(heading_diff) > abs(heading_diff_backward):
                self.moving_backward = True
                heading_diff = heading_diff_backward
            else:
                self.moving_backward = False
        if abs(heading_diff) > self.heading_tolerance:
            self._rotate_in_place(theta_error=heading_diff)
        else:
            self._move_forward(pos_error=dist, theta_error=heading_diff)

    def _rotate_in_place(self, theta_error=1.0):
        theta_vel_raw = theta_error * self.p_theta_in_place
        theta_vel = Utils.clip(theta_vel_raw, self.max_theta_vel, self.min_theta_vel)
        self._cmd_vel_pub.publish(Utils.get_twist(x=0.0, y=0.0, theta=theta_vel))

    def _move_forward(self, pos_error=1.0, theta_error=1.0):
        future_vel_prop_raw = pos_error * self.p_linear
        future_vel_prop = Utils.clip(future_vel_prop_raw, self.max_linear_vel, self.min_linear_vel)

        theta_vel_raw = theta_error * self.p_theta
        theta_vel = Utils.clip(theta_vel_raw, self.max_theta_vel, self.min_theta_vel)

        num_of_points = 10
        future_points = Utils.get_future_positions(future_vel_prop,
                                                   theta_vel,
                                                   num_of_points,
                                                   self.future_pos_lookahead_time)
        collision_index = self._get_collision_index(future_points)

        if collision_index == 0:
            self.retry()
            return

        desired_x_vel = future_vel_prop * (float(collision_index)/num_of_points)
        desired_x_vel = Utils.clip(desired_x_vel, self.max_linear_vel, self.min_linear_vel)

        # ramp up the vel according to max_linear_acc
        x_vel = min(desired_x_vel, self.current_vel + self.max_linear_acc)

        if self.moving_backward:
            x_vel *= -1
            for p in future_points:
                p.x *= -1
        self._cmd_vel_pub.publish(Utils.get_twist(x=x_vel, y=0.0, theta=theta_vel))
        self._traj_pub.publish(self._get_path_msg_from_points(future_points))

    def retry(self):
        rospy.logerr('Obstacle ahead. Current plan failed.')
        if self.retry_attempts < self.num_of_retries:
            rospy.loginfo('Retrying')
            self.retry_attempts += 1
            self.publish_zero_vel()
            if self.recovery_enabled:
                self.recover()
            self.publish_nav_feedback(ManeuverNavFeedback.BUSY)
            self.plan = None
        else:
            rospy.logerr('ABORTING')
            self.publish_nav_feedback(ManeuverNavFeedback.FAILURE_OBSTACLES)
            self._reset_state()

    def recover(self):
        rospy.loginfo('Recovering')
        self.max_linear_vel = rospy.get_param('~max_linear_vel', 0.3)
        if self.recovery_wait_duration > 0:
            rospy.loginfo('Recovery bahaviour: WAITING (duration: ' +
                          str(self.recovery_wait_duration) + ' seconds)')
            rospy.sleep(self.recovery_wait_duration)
        if self.recovery_motion_duration > 0:
            rospy.loginfo('Recovery bahaviour: MOVE BACKWARDS (duration: ' +
                          str(self.recovery_motion_duration) + ' seconds)')
            start_time = rospy.get_time()
            recovery_twist = Utils.get_twist(x=self.recovery_vel)
            if self.moving_backward:
                recovery_twist.linear.x *= -1
            while rospy.get_time() < start_time + self.recovery_motion_duration:
                rospy.sleep(0.1)
                self._cmd_vel_pub.publish(recovery_twist)

        rospy.loginfo('Recovery finished')

    def goal_cb(self, msg):
        if self.plan is not None:
            rospy.logwarn('Preempting previous goal. User requested another goal')
        self._reset_state()
        self.goal = Utils.get_x_y_theta_from_pose(msg.pose)
        rospy.loginfo('Received new goal')
        rospy.loginfo(self.goal)

    def path_cb(self, msg):
        # sanity checks
        if self.global_frame != msg.header.frame_id:
            rospy.logwarn('Goal path has "' + msg.header.frame_id + '" frame. '\
                          + 'Expecting "' + self.global_frame + '". Ignoring.')
            return
        path = msg.poses
        if len(path) == 0:
            rospy.logwarn('Received empty goal path. Ignoring')
            return
        first_pose = Utils.get_x_y_theta_from_pose(path[0].pose)
        dist = Utils.get_distance_between_points(first_pose[:2], self.curr_pos[:2])
        if dist > self.goal_path_start_point_tolerance:
            rospy.logwarn('Goal path first point is too far from robot. Ignoring.')
            return

        if len(path) == 2:
            self.max_linear_vel = rospy.get_param('~max_linear_vel_fast', 0.3)
        else:
            self.max_linear_vel = rospy.get_param('~max_linear_vel', 0.3)
        print(self.max_theta_vel)
        self.plan = path
        self.goal = Utils.get_x_y_theta_from_pose(self.plan[-1].pose)
        self._path_pub.publish(msg)

    def odom_cb(self, msg):
        self.current_vel = msg.twist.twist.linear.x

    def laser_cb(self, msg):
        angle_inc = msg.angle_increment

        if msg.angle_min > self.min_angle_front or msg.angle_max < self.max_angle_front:
            rospy.logerr("laser message data not available in filter range")

        first_index = int((self.min_angle_front - msg.angle_min)/angle_inc)
        last_index = int((self.max_angle_front - msg.angle_min )/angle_inc)
        filtered_ranges = msg.ranges[first_index:last_index]
        self.min_laser_dist_front = min(filtered_ranges)

        filtered_ranges_back = []
        if self.allow_backward_motion:
            if msg.angle_min > self.min_angle_back or msg.angle_max < self.max_angle_back:
                rospy.logerr("laser message data not available in filter range")

            first_index = int((self.min_angle_back - msg.angle_min)/angle_inc)
            last_index = int((self.max_angle_back - msg.angle_min )/angle_inc)
            filtered_ranges_back = msg.ranges[first_index:last_index]
            self.min_laser_dist_back = min(filtered_ranges_back)

        filtered_laser_msg = LaserScan()
        filtered_laser_msg.header = msg.header
        filtered_laser_msg.angle_min = self.min_angle_front
        filtered_laser_msg.angle_max = self.max_angle_front
        filtered_laser_msg.angle_increment = msg.angle_increment
        filtered_laser_msg.range_min = msg.range_min
        filtered_laser_msg.range_max = msg.range_max
        filtered_laser_msg.ranges = filtered_ranges_back if self.moving_backward else filtered_ranges

        self._laser_pub.publish(filtered_laser_msg)

    def get_current_position_from_tf(self):
        try:
            trans, rot = self._tf_listener.lookupTransform(self.global_frame,
                                                           self.robot_frame,
                                                           rospy.Time(0))
            _, _, yaw = tf.transformations.euler_from_quaternion(rot)
            self.curr_pos = (trans[0], trans[1], yaw)
        except Exception as e:
            rospy.logerr(str(e))
            self.curr_pos = None

    def _get_global_plan(self):
        """Call global planner to get a plan based on current position and goal

        :returns: None

        """
        self.plan = self.global_planner_utils.get_global_plan(self.curr_pos, self.goal)
        if self.plan is None:
            rospy.logerr('Global planner failed.')
            if self.retry_attempts < self.num_of_retries:
                rospy.loginfo('Retrying')
                self.retry_attempts += 1
                self.publish_zero_vel()
                if self.recovery_enabled:
                    self.recover()
                self.publish_nav_feedback(ManeuverNavFeedback.BUSY)
            else:
                rospy.logerr('ABORTING')
                self.publish_nav_feedback(ManeuverNavFeedback.FAILURE_OBSTACLES)
                self._reset_state()
            return

        # publish path
        path_msg = Path()
        path_msg.header.frame_id = self.global_frame
        path_msg.header.stamp = rospy.Time.now()
        path_msg.poses = self.plan
        self._path_pub.publish(path_msg)

    def _get_straight_line_plan(self):
        """
        Generate a straight line path to reach goal
        """
        self.plan = []
        path_msg = Path()
        path_msg.header.frame_id = self.global_frame
        path_msg.header.stamp = rospy.Time.now()

        start_pose = Utils.get_pose_stamped_from_frame_x_y_theta(self.global_frame,
                                                                 *self.curr_pos)
        goal_pose = Utils.get_pose_stamped_from_frame_x_y_theta(self.global_frame,
                                                                *self.goal)
        self.plan.append(start_pose)
        self.plan.append(goal_pose)
        path_msg.poses = self.plan

        self._path_pub.publish(path_msg)

    def _get_collision_index(self, points):
        if self.moving_backward:
            obs_dist_from_base = self.min_laser_dist_back - self.base_link_laser_dist - self.safety_dist
        else:
            obs_dist_from_base = self.min_laser_dist_front + self.base_link_laser_dist - self.safety_dist

        for i in range(len(points)):
            if points[i].x > obs_dist_from_base:
                return i
        return len(points)

    def _get_path_msg_from_points(self, points):
        path_msg = Path()
        path_msg.header.stamp = rospy.Time.now()
        path_msg.header.frame_id = self.robot_frame

        for p in points:
            pose = PoseStamped()
            pose.pose.position = p
            pose.pose.orientation.w = 1.0
            path_msg.poses.append(pose)
        return path_msg

    def _reset_state(self):
        self.max_linear_vel = rospy.get_param('~max_linear_vel', 0.3)
        self.publish_zero_vel()
        self.goal = None
        self.plan = None
        self.reached_goal_once = False
        self.moving_backward = False
        self.retry_attempts = 0

    def cancel_current_goal(self, msg):
        rospy.logwarn('PREEMPTING (cancelled goal)')
        self.publish_nav_feedback(ManeuverNavFeedback.FAILURE_OBSTACLES)
        self._reset_state()

    def publish_zero_vel(self):
        self._cmd_vel_pub.publish(Utils.get_twist())

    def publish_nav_feedback(self, status):
        """
        Publish feedback for higher level components to manage/track the progress
        """
        # TODO: dist_to_obs is used as distance to goal. Should be changed
        if status == ManeuverNavFeedback.BUSY:
            dist = Utils.get_distance_between_points(self.goal[:2], self.curr_pos[:2])
        else:
            dist = 0.0
        self._nav_feedback_pub.publish(ManeuverNavFeedback(status=status, dist_to_obs=dist))

