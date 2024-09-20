#!/usr/bin/env python3
import rospy
from geometry_msgs.msg import Twist
from sensor_msgs.msg import Imu
from SunFounder_Line_Follower import Line_Follower
from picar import back_wheels, front_wheels
import picar

picar.setup()

# Constants
forward_speed = 50
turning_speed = 45
stop_threshold = 0.3
timeout_duration = 1.0
turning_angle = 30
backup_speed = 40          # Speed for backward movement
backup_duration = 2.0       # Duration for backward movement (seconds)

# Initialize Line Follower and IMU
lf = Line_Follower.Line_Follower()

# Wheel Initialization
bw = back_wheels.Back_Wheels()
fw = front_wheels.Front_Wheels()
bw.ready()
fw.ready()
fw.turning_max = 45

last_cmd_vel_time = None
obstacle_detected = False
imu_data = None
line_lost = False

def move_forward(speed):
    bw.speed = speed
    bw.forward()

def move_backward(speed):
    bw.speed = speed
    bw.backward()

def stop():
    bw.stop()

def detect_line():
    try:
        values = lf.read_analog()
        rospy.loginfo("Sensor values: {}".format(values))

        avg_value = sum(values) / len(values)
        dynamic_threshold = avg_value * 0.7

        if all(value > dynamic_threshold for value in values):
            return False
        return True
    except Exception as e:
        rospy.logerr("Error occurred: {}".format(e))
        stop()
        return False

def imu_callback(data):
    global imu_data
    imu_data = data

def cmd_vel_callback(data):
    global last_cmd_vel_time, line_lost
    last_cmd_vel_time = rospy.get_time()

    linear_vel = data.linear.x
    angular_vel = data.angular.z

    if detect_line():
        line_lost = False
        try:
            values = lf.read_analog()
            lowest_index = values.index(min(values))
            roll, pitch, yaw = (0, 0, 0)
            if imu_data:
                roll = imu_data.linear_acceleration.x
                pitch = imu_data.linear_acceleration.y
                yaw = imu_data.angular_velocity.z
            rospy.loginfo("IMU orientation: roll={}, pitch={}, yaw={}".format(roll, pitch, yaw))

            if lowest_index == 0:
                fw.turn(50)
                move_forward(turning_speed)
            elif lowest_index == 1:
                fw.turn(70)
                move_forward(turning_speed)
            elif lowest_index == 2:
                fw.turn(90)
                move_forward(forward_speed)
            elif lowest_index == 3:
                fw.turn(110)
                move_forward(turning_speed)
            elif lowest_index == 4:
                fw.turn(130)
                move_forward(turning_speed)
            else:
                stop()

            # Adjust heading based on IMU data
            if roll > 10 or pitch > 10:
                fw.turn(90 + yaw)
                move_forward(turning_speed)

        except Exception as e:
            rospy.logerr("Error occurred: {}".format(e))
            stop()
    else:
        # Line is lost, move backward and then try to readjust
        if not line_lost:
            line_lost = True
            rospy.logwarn("Line lost! Moving backward to readjust.")
            move_backward(backup_speed)
            rospy.sleep(backup_duration)
            stop()
        else:
            stop()

def check_cmd_vel_timeout(event):
    global last_cmd_vel_time
    if last_cmd_vel_time is not None:
        current_time = rospy.get_time()
        elapsed_time = current_time - last_cmd_vel_time
        if elapsed_time > timeout_duration:
            stop()

def driver_motor_node():
    rospy.init_node('driver_motor')
    rospy.Subscriber('cmd_vel', Twist, cmd_vel_callback)
    rospy.Subscriber('imu/data', Imu, imu_callback)
    rospy.Timer(rospy.Duration(0.1), check_cmd_vel_timeout)
    rospy.spin()

def shutdown_hook():
    stop()
    rospy.loginfo("Stopping the motor...")

if __name__ == '__main__':
    try:
        rospy.on_shutdown(shutdown_hook)
        driver_motor_node()
    except rospy.ROSInterruptException:
        rospy.loginfo("Ctrl+C detected. Stopping the node...")
        stop()
