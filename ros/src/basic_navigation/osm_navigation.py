from __future__ import print_function

import tf
import time
import math
import rospy
import traceback

from std_msgs.msg import String, Empty
from geometry_msgs.msg import PoseStamped, PointStamped, Point
from nav_msgs.msg import Path
from basic_navigation.msg import BasicNavigationFeedback as Feedback

from utils import Utils
from topological_planner import TopologicalPlanner
from geometric_planner import GeometricPlanner

class OSMNavigation(object):

    """Navigation using OSM map"""

    def __init__(self):
        # ROS params
        self.global_frame = rospy.get_param('~global_frame', 'map')
        self.robot_frame = rospy.get_param('~robot_frame', 'load/base_link')
        self.geometric_wp_goal_tolerance = rospy.get_param('~geometric_wp_goal_tolerance', 0.5)
        self.topological_wp_goal_tolerance = rospy.get_param('~topological_wp_goal_tolerance', 1.0)
        self.plan_next_geometric_path_tolerance = rospy.get_param('~plan_next_geometric_path_tolerance', 2)
        network_file = rospy.get_param('~network_file', None)

        # class variables
        self.tf_listener = tf.TransformListener()
        self.topological_planner = TopologicalPlanner(network_file)
        self.geometric_planner = GeometricPlanner(tf_listener=self.tf_listener)
        self.topological_path = None
        self.geometric_path = None
        self.plan_next_geometric_path = False
        self.replan_current_geometric_path = False

        # subscribers
        clicked_point_sub = rospy.Subscriber('/clicked_point', PointStamped, self.goal_cb)
        cancel_goal_sub = rospy.Subscriber('~cancel', Empty, self.cancel_current_goal)
        feedback_sub = rospy.Subscriber('~bn_feedback', Feedback, self.feedback_cb)

        # publishers
        self._path_pub = rospy.Publisher('~topological_path', Path, queue_size=1)
        self._cancel_bn_pub = rospy.Publisher('~cancel_bn', Empty, queue_size=1)
        self._bn_path_pub = rospy.Publisher('~bn_goal_path', Path, queue_size=1)
        self._bn_mode_pub = rospy.Publisher('~bn_mode_path', String, queue_size=1)

        rospy.loginfo('Initialised')

    def run_once(self):
        """
        Main control loop
        """
        if self.topological_path is None:
            return

        if self.geometric_path is None:
            self.geometric_path = self._get_geometric_path(goal_pose=self.topological_path[0])
            return

        if self.plan_next_geometric_path:
            print('\n\nReached a topological WP\n\n')
            self.plan_next_geometric_path = False
            curr_goal = self.topological_path.pop(0)
            if len(self.topological_path) != 0:
                geometric_path = self._get_geometric_path(start_pose=curr_goal,
                                                          goal_pose=self.topological_path[0])
                self.geometric_path.extend(geometric_path)
            return

    def feedback_cb(self, msg):
        if self.topological_path is None:
            return
        if msg.status == Feedback.SUCCESS:
            if len(self.topological_path) == 0:
                print('\n\nREACHED GOAL\n\n')
                self.topological_path = None
            else:
                rospy.logerr('OOPS! This should never happen!') # TODO call recovery manager?
                self._reset_state()
        elif msg.status == Feedback.REACHED_WP:
            wp = Utils.get_x_y_theta_from_pose(msg.reached_wp)
            geometric_path_wp = Utils.get_x_y_theta_from_pose(self.geometric_path[0].pose)
            dist = Utils.get_distance_between_points(wp[:2], geometric_path_wp)
            if dist < self.geometric_wp_goal_tolerance and msg.remaining_path_length == len(self.geometric_path)-1: # verify
                rospy.loginfo('In sync with Basic navigation')
                self.geometric_path.pop(0)
            else:
                rospy.logerr('Lost sync with Basic navigation. Aborting') # TODO call recovery manager?
                print('dist', dist)
                print('msg remaining path length', msg.remaining_path_length)
                print('osm beleived remaining path length', len(self.geometric_path))
                print('osm wp', geometric_path_wp)
                print('msg wp', wp)
                self._reset_state()
            self.plan_next_geometric_path = msg.remaining_path_length < self.plan_next_geometric_path_tolerance
        elif msg.status == Feedback.FAILURE_OBSTACLES:
            pass # TODO call recovery manager
        elif msg.status == Feedback.FAILURE_EMPTY_PLAN or msg.status == Feedback.FAILURE_INVALID_PLAN:
            self.geometric_path = None
        elif msg.status == Feedback.FAILURE_NO_CURRENT_POSE:
            rospy.logerr('Basic navigation could not find current pose')
            # TODO call recovery manager
        elif msg.status == Feedback.CANCELLED:
            rospy.logerr('Basic navigation was cancelled')
            rospy.logerr('CANCELLING')
            self.topological_path = None
            self._reset_state()

    def goal_cb(self, msg):
        if self.topological_path is not None:
            rospy.logwarn('Preempting ongoing path. User requested another goal')
        self._reset_state()
        goal = (msg.point.x, msg.point.y)
        rospy.loginfo('Received new goal')
        rospy.loginfo(goal)
        self._get_osm_path(goal)
        if len(self.topological_path) == 2:
            self._bn_mode_pub.publish(String(data='lenient'))
        else:
            self._bn_mode_pub.publish(String(data='long_dist'))
        print('len of topological plan', len(self.topological_path))

    def cancel_current_goal(self, msg):
        """Cancel current goal by sending a cancel signal to basic navigation

        :msg: std_msgs.Empty
        :returns: None

        """
        rospy.logwarn('Preempting')
        self._reset_state()

    def _reset_state(self):
        if self.topological_path is not None:
            self._cancel_bn_pub.publish(Empty())
        self.topological_path = None
        self.geometric_path = None
        self.plan_next_geometric_path = False
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

    def _get_osm_path(self, goal):
        """
        Call topological planner to get point plan from current position to goal pos
        Generates orientation for point plan to create Path msg 
        Publishes Path msg for visualisation

        :goal: tuple of 2 float
        :returns: None

        """
        curr_pos = self.get_current_position_from_tf()
        if curr_pos is None:
            rospy.logerr('Cannot get current position of the robot')
            return None
        rospy.loginfo('Current pos: ' + str(curr_pos))
    
        plan = self.topological_planner.plan(curr_pos[:2], goal[:2])
        if plan is None or len(plan) == 0:
            rospy.logerr('Could not plan topological plan.')
            self._reset_state()
            return

        path_msg = Path()
        path_msg.header.frame_id = self.global_frame
        path_msg.header.stamp = rospy.Time.now()

        theta = 0.0
        self.topological_path = []
        for i in range(len(plan)):
            if i < len(plan)-1:
                theta = math.atan2(plan[i+1].y - plan[i].y, plan[i+1].x - plan[i].x)
            pose = (plan[i].x, plan[i].y, theta)
            pose_stamped = Utils.get_pose_stamped_from_frame_x_y_theta(self.global_frame, *pose)
            self.topological_path.append(pose)
            path_msg.poses.append(pose_stamped)
        self._path_pub.publish(path_msg)
        self.topological_path.pop(0) # remove current position
        rospy.loginfo('Planned topological path successfully')

    def _get_geometric_path(self, goal_pose, start_pose=None):
        """
        Call geometric planner to get plan from current position to goal pos
        Publishes Path msg for visualisation

        :goal_pose: tuple of 3 float
        :start_pose: tuple of 3 float
        :returns: None

        """
        if start_pose is None:
            curr_pos = self.get_current_position_from_tf()
            if curr_pos is None:
                rospy.logerr('Cannot get current position of the robot')
                return None
            rospy.loginfo('Current pos: ' + str(curr_pos))
            start_pose = curr_pos
    
        plan = self.geometric_planner.plan(start_pose, goal_pose)
        if plan is None or len(plan) == 0:
            rospy.logerr('Could not plan geometric plan.')
            # TODO call recovery manager
            return None

        path_msg = Utils.get_path_msg_from_poses(plan, self.global_frame)
        geometric_path = path_msg.poses
        self._bn_path_pub.publish(path_msg)
        rospy.loginfo('Planned path successfully')
        return geometric_path
