#!/usr/bin/env python3
import rospy
from line_follower_robot.msg import LineFollowerStatus  # Custom message type
from geometry_msgs.msg import Twist
from mpu6050 import mpu6050  # Assuming you're using an mpu6050 Python library

class LineFollowerNode:
    def __init__(self):
        rospy.init_node('line_follower_node')
        # Subscribers and Publishers
        rospy.Subscriber('lf_status', LineFollowerStatus, self.line_follow_callback)
        self.cmd_vel_pub = rospy.Publisher('cmd_vel', Twist, queue_size=1)
        # Initialize MPU6050
        self.mpu = mpu6050(0x68)
        self.twist_msg = Twist()

        # Control parameters
        self.linear_vel = 0.2
        self.angular_vel = 0.0
        self.gyro_threshold = 20  # Threshold to detect sharp turns based on gyro z-axis

    def read_imu(self):
        accel_data = self.mpu.get_accel_data()
        gyro_data = self.mpu.get_gyro_data()
        return accel_data, gyro_data

    def line_follow_callback(self, data):
        lt_status = data.status
        rospy.loginfo("Received lt_status: %s", lt_status)

        # Initialize control variables
        angular_vel = 0.0
        # IMU Data
        accel_data, gyro_data = self.read_imu()
        # Proportional control logic based on IR sensors
        if lt_status:
            center_sensor = lt_status[2]
            left_sensors = lt_status[:2]
            right_sensors = lt_status[3:]

            if center_sensor == 1:
                angular_vel = 0
            else:
                left_sum = sum(left_sensors)
                right_sum = sum(right_sensors)
                error = left_sum - right_sum
                angular_vel = -0.1 * error
        # If all sensors are off the line, initiate the backup and readjustment
        if sum(lt_status) == 0:
            rospy.logwarn("All sensors off the line. Sending backup command.")
            self.twist_msg.linear.x = -0.2  # Command to move backward
            self.twist_msg.angular.z = 0.0  # Keep it straight while moving backward
        else:
            # Adjust angular velocity based on gyro data (to handle sharp turns)
            if abs(gyro_data['z']) > self.gyro_threshold:
                rospy.loginfo("Sharp turn detected! Adjusting angular velocity.")
                if gyro_data['z'] > 0:
                    angular_vel -= 0.2  # Adjust to turn right
                else:
                    angular_vel += 0.2  # Adjust to turn left

            # Set the velocities
            self.twist_msg.linear.x = self.linear_vel
            self.twist_msg.angular.z = angular_vel
        # Publish the Twist message
        self.cmd_vel_pub.publish(self.twist_msg)

if __name__ == '__main__':
    try:
        line_follower_node = LineFollowerNode()
        rospy.spin()
    except rospy.ROSInterruptException:
        rospy.loginfo("Ctrl+C detected. Stopping the node...")
