#!/usr/bin/env python3

import rospy
from SunFounder_Line_Follower import Line_Follower
from std_msgs.msg import String

class GetLineNode:
    def __init__(self):
        rospy.init_node('get_line')
        self.pub = rospy.Publisher("lf_status", String, queue_size=1)
        self.lf = Line_Follower.Line_Follower()
        self.rate = rospy.Rate(30)
        self.read_data()

    def read_data(self):
        rospy.loginfo("Start reading")
        while not rospy.is_shutdown():
            lt_status = self.lf.read_analog()
            rospy.loginfo(lt_status)
            self.pub.publish(String(str(lt_status)))
            self.rate.sleep()

if __name__ == "__main__":
    try:
        get_line_node = GetLineNode()
        rospy.spin()
    except rospy.ROSInterruptException:
        rospy.loginfo("Ctrl+C detected. Stopping the node...")
