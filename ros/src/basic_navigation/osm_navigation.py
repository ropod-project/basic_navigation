from __future__ import print_function

import tf
import time
import math
import rospy
import traceback
from OBL import OSMBridge, PathPlanner

from std_msgs.msg import String, Empty
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Path
from maneuver_navigation.msg import Feedback as ManeuverNavFeedback

from utils import Utils
from global_planner_utils import GlobalPlannerUtils

class OSMNavigation(object):

    """Navigation using OSM map"""

    def __init__(self):
        # OSM related ros params and variables
        ref_latitude = rospy.get_param('~ref_latitude', None)
        ref_longitude = rospy.get_param('~ref_longitude', None)
        if ref_latitude is None or ref_longitude is None:
            rospy.logfatal('Reference global origin not provided. Exiting.')
            sys.exit(1)
        self.osm_bridge = OSMBridge(global_origin=[ref_latitude, ref_longitude])

        building = rospy.get_param('~building', 'BRSU')
        self.floor_prefix = building + '_L'
        self.path_planner = PathPlanner(self.osm_bridge)
        self.path_planner.set_building(building)

        # ROS params
        self.global_frame = rospy.get_param('~global_frame', 'map')
        self.robot_frame = rospy.get_param('~robot_frame', 'load/base_link')
        self.wp_goal_tolerance = rospy.get_param('~wp_goal_tolerance', 1.0)

        # class variables
        self.tf_listener = tf.TransformListener()
        self.global_planner_utils = GlobalPlannerUtils()
        self.path = None
        self.goal = None
        self.bn_reached_curr_wp = None

        # subscribers
        goal_sub = rospy.Subscriber('~goal', String, self.goal_cb)
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
                self._get_osm_path(self.goal)
                self.create_plan_and_send()
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
        self.goal = msg.data
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
            # global_planner_path = self.global_planner_utils.get_global_plan(start_pos, new_wp)
            # if global_planner_path is not None:
            #     path_msg = Path(poses=global_planner_path)
            #     path_msg.header.stamp = rospy.Time.now()
            #     path_msg.header.frame_id = self.global_frame
            #     self._bn_path_pub.publish(path_msg)
            #     rospy.loginfo('Sent new path')
            #     self.bn_reached_curr_wp = False
            #     return
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

    def _get_osm_path(self, dest_local_area):
        """
        Tries to get a osm path from current location to `dest_local_area`
        `dest_local_area` can be either a string or an int written as string
        e.g.  "BRSU_C_L0_C7_LA1" or "66"

        :dest_local_area: string or int
        :returns: None

        """
        curr_pos = self.get_current_position_from_tf()
        if curr_pos is None:
            return None
        x, y, _ = curr_pos
        rospy.loginfo('Current pos: ' + str((x, y)))

        current_floor = rospy.get_param('~floor', 0)
        start_floor = self.floor_prefix + str(current_floor)
        start_local_area_obj = self.osm_bridge.get_local_area(x=x, y=y,
                                                              floor_name=start_floor,
                                                              isLatlong=False)
        try:
            dest_local_area = int(dest_local_area)
        except ValueError:
            pass

        dest_local_area_obj = self.osm_bridge.get_local_area(dest_local_area)
        dest_local_area_obj.geometry
        dest_area = dest_local_area_obj.parent_id
        dest_floor = self.floor_prefix + str(dest_local_area_obj.level)

        rospy.loginfo('Trying to plan a path from ' + str(start_local_area_obj.ref) \
                        + ' to ' + str(dest_local_area_obj.ref))
        osm_path = self.path_planner.get_path_plan(start_floor=start_floor,
                                                   destination_floor=dest_floor,
                                                   start_area=start_local_area_obj.parent_id,
                                                   destination_area=dest_area,
                                                   start_local_area=start_local_area_obj.id,
                                                   destination_local_area=dest_local_area)

        path_msg = Path()
        path_msg.header.frame_id = self.global_frame
        path_msg.header.stamp = rospy.Time.now()

        pos_path = [[p.navigation_areas[0].topology.x,
                     p.navigation_areas[0].topology.y] for p in osm_path]
        pos_path.insert(0, [x, y]) # insert current position

        theta = 0.0
        self.path = []
        for i in range(len(pos_path)):
            if i < len(pos_path)-1:
                theta = math.atan2(pos_path[i+1][1] - pos_path[i][1],
                                   pos_path[i+1][0] - pos_path[i][0])
            pos_path[i].append(theta)
            pose = Utils.get_pose_stamped_from_frame_x_y_theta(self.global_frame,
                                                               *pos_path[i])
            self.path.append(pose)

        path_msg.poses = self.path

        self._path_pub.publish(path_msg)
        if len(self.path) > 1:
            self.path.pop(0) # remove current location
            self.path.pop(0) # remove current area's wp
        rospy.loginfo('Planned path successfully')

