from __future__ import print_function

import rospy

class RecoveryManager(object):

    """Monitor and execute recovery behaviours"""

    def __init__(self):
        self.recovery_wait_duration = rospy.get_param('~recovery_wait_duration', 1.0)
        self.recovery_motion_duration = rospy.get_param('~recovery_motion_duration', 2.0)
        self.recovery_vel = rospy.get_param('~recovery_vel', -0.1)
        
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

