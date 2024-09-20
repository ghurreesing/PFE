#!/usr/bin/env python3
import rospy
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Imu
import tf
import math

class OdometryPublisher:
    def __init__(self):
        rospy.init_node('odometry_publisher')

        self.odom_pub = rospy.Publisher('/odom', Odometry, queue_size=10)
        self.odom_broadcaster = tf.TransformBroadcaster()

        self.current_time = rospy.Time.now()
        self.last_time = rospy.Time.now()

        self.x = 0.0
        self.y = 0.0
        self.th = 0.0

        self.vx = 0.0
        self.vy = 0.0
        self.vth = 0.0

        rospy.Subscriber('imu/data', Imu, self.imu_callback)

    def imu_callback(self, data):
        self.current_time = rospy.Time.now()

        #orientation
        orientation = data.orientation
        _, _, self.th = tf.transformations.euler_from_quaternion([orientation.x, orientation.y, orientation.z, orientation.w])

        #linear acceleration
        linear_acceleration_x = data.linear_acceleration.x
        linear_acceleration_y = data.linear_acceleration.y
        angular_velocity_z = data.angular_velocity.z

        dt = (self.current_time - self.last_time).to_sec()

        self.vx += linear_acceleration_x * dt
        self.vy += linear_acceleration_y * dt
        self.vth = angular_velocity_z

        # Update position
        delta_x = self.vx * dt * math.cos(self.th) - self.vy * dt * math.sin(self.th)
        delta_y = self.vx * dt * math.sin(self.th) + self.vy * dt * math.cos(self.th)
        
        self.x += delta_x
        self.y += delta_y

        odom_quat = tf.transformations.quaternion_from_euler(0, 0, self.th)

        self.odom_broadcaster.sendTransform(
            (self.x, self.y, 0.),
            odom_quat,
            self.current_time,
            "base_link",
            "odom"
        )

        odom = Odometry()
        odom.header.stamp = self.current_time
        odom.header.frame_id = "odom"

        odom.pose.pose.position.x = self.x
        odom.pose.pose.position.y = self.y
        odom.pose.pose.position.z = 0
        odom.pose.pose.orientation.x = odom_quat[0]
        odom.pose.pose.orientation.y = odom_quat[1]
        odom.pose.pose.orientation.z = odom_quat[2]
        odom.pose.pose.orientation.w = odom_quat[3]

        odom.child_frame_id = "base_link"
        odom.twist.twist.linear.x = self.vx
        odom.twist.twist.linear.y = self.vy
        odom.twist.twist.angular.z = self.vth

        # Publish odometry message
        self.odom_pub.publish(odom)

        self.last_time = self.current_time

    def run(self):
        rospy.spin()

if __name__ == '__main__':
    try:
        odometry_publisher = OdometryPublisher()
        odometry_publisher.run()
    except rospy.ROSInterruptException:
        rospy.loginfo("Ctrl+C detected. Stopping the node...")
