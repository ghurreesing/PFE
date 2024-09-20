#!/usr/bin/env python3
import rospy
from geometry_msgs.msg import Twist
from SunFounder_Line_Follower import Line_Follower
from picar import back_wheels, front_wheels
import picar

picar.setup()

# Constants
forward_speed = 60
stop_threshold = 4
turning_speed = 20  # Speed during turns
timeout_duration = 1.0

# Line Follower Initialization
lf = Line_Follower.Line_Follower()

# Wheel Initialization
bw = back_wheels.Back_Wheels()
fw = front_wheels.Front_Wheels()
bw.ready()
fw.ready()
fw.turning_max = 45

last_cmd_vel_time = None

def move_forward():
    bw.speed = forward_speed
    bw.forward()

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

def cmd_vel_callback(data):
    global last_cmd_vel_time
    last_cmd_vel_time = rospy.get_time()

    linear_vel = data.linear.x
    angular_vel = data.angular.z

    if detect_line():
        try:
            values = lf.read_analog()
            lowest_index = values.index(min(values))
            rospy.loginfo("Lowest index: {}".format(lowest_index))

            if lowest_index == 0:
                fw.turn(50)
                bw.speed = turning_speed
                move_forward()
            elif lowest_index == 1:
                fw.turn(70)
                bw.speed = turning_speed
                move_forward()
            elif lowest_index == 2:
                fw.turn(90)
                move_forward()
            elif lowest_index == 3:
                fw.turn(110)
                bw.speed = turning_speed
                move_forward()
            elif lowest_index == 4:
                fw.turn(130)
                bw.speed = turning_speed
                move_forward()
            else:
                stop()

        except Exception as e:
            rospy.logerr("Error occurred: {}".format(e))
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
    rospy.Timer(rospy.Duration(0.5), check_cmd_vel_timeout)
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

