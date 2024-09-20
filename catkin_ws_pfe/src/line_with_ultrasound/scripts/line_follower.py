#!/usr/bin/env python3
import rospy
from std_msgs.msg import String
from geometry_msgs.msg import Twist

class LineFollowerNode:
    def __init__(self):
        rospy.init_node('line_follower')
        rospy.Subscriber('lf_status', String, self.line_follow_callback)
        self.cmd_vel_pub = rospy.Publisher('cmd_vel', Twist, queue_size=1)
        self.twist_msg = Twist()

    def line_follow_callback(self, data):
        lt_status = data.data
        #rospy.loginfo("Received lt_status: %s", lt_status)
        if lt_status == 'on_track':
            linear_vel = 0.2
            angular_vel = 0.0
        else:
            linear_vel = 0.0
            angular_vel = 0.1
        self.twist_msg.linear.x = linear_vel
        self.twist_msg.angular.z = angular_vel
        self.cmd_vel_pub.publish(self.twist_msg)

if __name__ == '__main__':
    try:
        line_follower_node = LineFollowerNode()
        rospy.spin()
    except rospy.ROSInterruptException:
        rospy.loginfo("Ctrl+C detected. Stopping the node...")
