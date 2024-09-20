#!/usr/bin/env python3
import rospy
from sensor_msgs.msg import Imu
from mpu6050 import mpu6050
import math

class ImuPublisher:
    def __init__(self):
        rospy.init_node('imu_publisher', anonymous=False)
        self.pub = rospy.Publisher('imu/data', Imu, queue_size=10)
        self.rate = rospy.Rate(10)  
        self.imu_sensor = mpu6050(0x68)
        self.run()

    def run(self):
        while not rospy.is_shutdown():
            try:
                accel_data = self.imu_sensor.get_accel_data()
                gyro_data = self.imu_sensor.get_gyro_data()

                imu_msg = Imu()
                imu_msg.header.stamp = rospy.Time.now()
                imu_msg.header.frame_id = 'imu_link'

                imu_msg.linear_acceleration.x = accel_data['x'] * 9.80665
                imu_msg.linear_acceleration.y = accel_data['y'] * 9.80665
                imu_msg.linear_acceleration.z = accel_data['z'] * 9.80665

                imu_msg.angular_velocity.x = math.radians(gyro_data['x'])
                imu_msg.angular_velocity.y = math.radians(gyro_data['y'])
                imu_msg.angular_velocity.z = math.radians(gyro_data['z'])

                imu_msg.orientation_covariance[0] = -1

                # Publish IMU data
                self.pub.publish(imu_msg)

            except Exception as e:
                rospy.logerr(f"Error reading from IMU sensor: {e}")

            self.rate.sleep()

if __name__ == '__main__':
    try:
        ImuPublisher()
    except rospy.ROSInterruptException:
        pass
