from __future__ import print_function

import tf
import time
import math
import rospy
import traceback

from std_msgs.msg import String, Empty
from geometry_msgs.msg import PoseStamped, PointStamped, Point
from nav_msgs.msg import Path
from maneuver_navigation.msg import Feedback as ManeuverNavFeedback

from utils import Utils
from topological_planner import TopologicalPlanner
from geometric_planner import GeometricPlanner

class OSMNavigation(object):

    """Navigation using OSM map"""

    def __init__(self):
        # ROS params
        self.global_frame = rospy.get_param('~global_frame', 'map')
        self.robot_frame = rospy.get_param('~robot_frame', 'load/base_link')
        self.wp_goal_tolerance = rospy.get_param('~wp_goal_tolerance', 1.0)
        network_file = rospy.get_param('~network_file', None)

        # class variables
        self.tf_listener = tf.TransformListener()
        self.topological_planner = TopologicalPlanner(network_file)
        self.geometric_planner = GeometricPlanner(tf_listener=self.tf_listener)
        self.path = None
        self.goal = None
        self.bn_reached_curr_wp = None

        # subscribers
        clicked_point_sub = rospy.Subscriber('/clicked_point', PointStamped, self.goal_cb)
        cancel_goal_sub = rospy.Subscriber('~cancel', Empty, self.cancel_current_goal)
        bn_feedback_sub = rospy.Subscriber('~bn_feedback', ManeuverNavFeedback,
                                           self.bn_feedback_cb)

        # publishers
        self._path_pub = rospy.Publisher('~path', Path, queue_size=1)
        self._cancel_bn_pub = rospy.Publisher('~cancel_bn', Empty, queue_size=1)
        self._bn_wp_pub = rospy.Publisher('~bn_goal', PoseStamped, queue_size=1)
        self._bn_path_pub = rospy.Publisher('~bn_goal_path', Path, queue_size=1)

        rospy.loginfo('Initialised')

    def run_once(self):
        """
        Main control loop
        """
        if self.goal is None:
            return

        if self.goal is not None and self.path is None:
            try:
                # self._get_osm_path()
                self._get_geometric_path()
                # self.create_plan_and_send()
            except Exception as e:
                rospy.logerr('Caught following Exception\n\n')
                traceback.print_exc()
                rospy.logerr('Ignoring goal\n')
                self._reset_state()

        if self.bn_reached_curr_wp:
            old_wp = self.path.pop(0)
            if len(self.path) == 0:
                rospy.loginfo('\n\nREACHED GOAL\n\n')
                self._reset_state()
                return
            self.create_plan_and_send(old_wp)

    def bn_feedback_cb(self, msg):
        if self.path is None:
            return
        if msg.status == ManeuverNavFeedback.SUCCESS:
            self.bn_reached_curr_wp = True
        elif msg.status == ManeuverNavFeedback.BUSY:
            if msg.dist_to_obs < self.wp_goal_tolerance and len(self.path) > 1:
                self.bn_reached_curr_wp = True
        elif msg.status >= ManeuverNavFeedback.FAILURE_OBSTACLES: # any type of failure
            self._reset_state()

    def goal_cb(self, msg):
        self.goal = (msg.point.x, msg.point.y, 0.0)
        rospy.loginfo('Received new goal')
        if self.path is not None:
            self.path = None
            rospy.logwarn('Preempting previous goal. User requested another goal')

    def create_plan_and_send(self, old_wp=None):
        if old_wp is None:
            start_pos = self.get_current_position_from_tf()
        else:
            start_pos = Utils.get_x_y_theta_from_pose(old_wp.pose)
        if start_pos is not None:
            new_wp = Utils.get_x_y_theta_from_pose(self.path[0].pose)

            plan = []
            path_msg = Path()
            path_msg.header.frame_id = self.global_frame
            path_msg.header.stamp = rospy.Time.now()

            start_pose = Utils.get_pose_stamped_from_frame_x_y_theta(self.global_frame,
                                                                     *start_pos)
            goal_pose = Utils.get_pose_stamped_from_frame_x_y_theta(self.global_frame,
                                                                    *new_wp)
            plan.append(start_pose)
            plan.append(goal_pose)
            path_msg.poses = plan

            self._bn_path_pub.publish(path_msg)
            rospy.loginfo('Sent new path')
            self.bn_reached_curr_wp = False
            return
        self._bn_wp_pub.publish(self.path[0])
        rospy.loginfo('Sent new WP')
        self.bn_reached_curr_wp = False

    def cancel_current_goal(self, msg):
        """Cancel current goal by sending a cancel signal to basic navigation

        :msg: std_msgs.Empty
        :returns: None

        """
        rospy.logwarn('Preempting')
        self._reset_state()

    def _reset_state(self):
        if self.goal is not None:
            self._cancel_bn_pub.publish(Empty())
        self.goal = None
        self.path = None
        self.bn_reached_curr_wp = False
        rospy.sleep(0.5)
        
    def get_current_position_from_tf(self):
        try:
            trans, rot = self.tf_listener.lookupTransform(self.global_frame,
                                                          self.robot_frame,
                                                          rospy.Time(0))
            _, _, yaw = tf.transformations.euler_from_quaternion(rot)
            curr_pos = (trans[0], trans[1], yaw)
        except Exception as e:
            rospy.logerr(str(e))
            curr_pos = None
        return curr_pos

    def _get_osm_path(self):
        """
        Call topological planner to get point plan from current position to goal pos
        Generates orientation for point plan to create Path msg 
        Publishes Path msg for visualisation

        :returns: None

        """
        curr_pos = self.get_current_position_from_tf()
        if curr_pos is None:
            rospy.logerr('Cannot get current position of the robot')
            return None
        rospy.loginfo('Current pos: ' + str(curr_pos))
    
        plan = self.topological_planner.plan(curr_pos[:2], self.goal[:2])
        if plan is None or len(plan) == 0:
            rospy.logerr('Could not plan topological plan.')
            self._reset_state()
            return

        path_msg = Path()
        path_msg.header.frame_id = self.global_frame
        path_msg.header.stamp = rospy.Time.now()

        theta = 0.0
        self.path = []
        for i in range(len(plan)):
            if i < len(plan)-1:
                theta = math.atan2(plan[i+1].y - plan[i].y, plan[i+1].x - plan[i].x)
            pose = Utils.get_pose_stamped_from_frame_x_y_theta(self.global_frame,
                                                               plan[i].x,
                                                               plan[i].y,
                                                               theta)
            self.path.append(pose)

        path_msg.poses = self.path

        self._path_pub.publish(path_msg)
        rospy.loginfo('Planned path successfully')

    def _get_geometric_path(self):
        """
        Call geometric planner to get plan from current position to goal pos
        Publishes Path msg for visualisation

        :returns: None

        """
        curr_pos = self.get_current_position_from_tf()
        if curr_pos is None:
            rospy.logerr('Cannot get current position of the robot')
            return None
        rospy.loginfo('Current pos: ' + str(curr_pos))
    
        plan = self.geometric_planner.plan(curr_pos, self.goal)
        if plan is None or len(plan) == 0:
            rospy.logerr('Could not plan geometric plan.')
            self._reset_state()
            return

        path_msg = Path()
        path_msg.header.frame_id = self.global_frame
        path_msg.header.stamp = rospy.Time.now()

        self.path = []
        for i in range(len(plan)):
            pose = Utils.get_pose_stamped_from_frame_x_y_theta(self.global_frame,
                                                               plan[i][0],
                                                               plan[i][1],
                                                               plan[i][2])
            self.path.append(pose)

        path_msg.poses = self.path

        self._path_pub.publish(path_msg)
        rospy.loginfo('Planned path successfully')
