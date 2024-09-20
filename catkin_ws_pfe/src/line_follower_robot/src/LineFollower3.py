#!/usr/bin/env python3
import rospy
from geometry_msgs.msg import Twist
from line_follower_robot.msg import LineFollowerStatus
from sensor_msgs.msg import Imu
import tf
import math

class LineFollowerNode:
    def __init__(self):
        rospy.init_node('line_follower_node')

        # Publisher for robot velocity commands
        self.cmd_vel_pub = rospy.Publisher('cmd_vel', Twist, queue_size=1)

        # Subscriber for line follower sensor data
        rospy.Subscriber('lf_status', LineFollowerStatus, self.line_follower_callback)

        # Subscriber for IMU data (if needed for additional stability)
        rospy.Subscriber('imu/data', Imu, self.imu_callback)

        # Robot velocity and angular velocity
        self.linear_vel = 0.2
        self.angular_vel = 0.0
        self.orientation_z = 0.0  # Orientation from IMU

        self.rate = rospy.Rate(30)  # 30 Hz

    def imu_callback(self, data):
        orientation = data.orientation
        _, _, self.orientation_z = tf.transformations.euler_from_quaternion(
            [orientation.x, orientation.y, orientation.z, orientation.w]
        )
        # Use orientation if needed to correct heading in line-following logic

    def line_follower_callback(self, msg):
        sensor_values = msg.status
        rospy.loginfo(f"Sensor values: {sensor_values}")

        # Determine the robot's action based on sensor readings
        twist = Twist()
        
        if sensor_values == [0, 0, 1, 0, 0]:  # Centered on the line
            twist.linear.x = 0.2  # Move forward
            twist.angular.z = 0.0  # No turn
        elif sensor_values[:3] == [0, 1, 1]:  # Slightly left of the line
            twist.linear.x = 0.15  # Move forward slower
            twist.angular.z = -0.2  # Turn right
        elif sensor_values[2:] == [1, 1, 0]:  # Slightly right of the line
            twist.linear.x = 0.15  # Move forward slower
            twist.angular.z = 0.2  # Turn left
        elif sensor_values[1:4] == [1, 1, 1]:  # On a T-junction or at the end of the line
            twist.linear.x = 0.0  # Stop
            twist.angular.z = 0.0  # No turn
            rospy.logwarn("Stopping the robot: T-junction or end of line detected.")
        elif sensor_values == [1, 1, 1, 1, 1]:  # All sensors detect the line (completely off track)
            twist.linear.x = -0.1  # Backup slowly
            twist.angular.z = 0.3  # Turn to search for the line
            
        else:  # If no sensors detect the line (likely off the line)
            twist.linear.x = 0.0  # Stop moving forward
            twist.angular.z = 0.3  # Spin to find the line again

        # Publish the velocity command
        self.cmd_vel_pub.publish(twist)

    def run(self):
        rospy.spin()

if __name__ == '__main__':
    try:
        node = LineFollowerNode()
        node.run()
    except rospy.ROSInterruptException:
        rospy.loginfo("Line follower node terminated.")
