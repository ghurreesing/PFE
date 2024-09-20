#!/usr/bin/env python3
import rospy
from SunFounder_Line_Follower import Line_Follower
from line_follower_robot.msg import LineFollowerStatus

class LineFollowerNode:
    def __init__(self):
        rospy.init_node('line_follower_node')
        self.pub = rospy.Publisher("lf_status", LineFollowerStatus, queue_size=10)
        self.lf = Line_Follower.Line_Follower()
        self.rate = rospy.Rate(30)
        self.read_data()

    def read_data(self):
        rospy.loginfo("Start reading line follower data")
        while not rospy.is_shutdown():
            try:
                lt_status = self.lf.read_analog()
                rospy.logdebug(f"Line follower status: {lt_status}")
                msg = LineFollowerStatus(status=lt_status)
                self.pub.publish(msg)
            except Exception as e:
                rospy.logerr(f"Error reading line follower data: {e}")
            self.rate.sleep()

if __name__ == "__main__":
    try:
        LineFollowerNode()
    except rospy.ROSInterruptException:
        rospy.loginfo("Ctrl+C detected. Stopping the node...")
