#!/usr/bin/env python3

import rospy
from geometry_msgs.msg import Twist
from SunFounder_Line_Follower import Line_Follower
from picar import back_wheels, front_wheels
import picar

# Initial setup for Picar
picar.setup()
REFERENCES = [17, 17, 17, 17, 17]
stop_threshold = 4
forward_speed = 40
lf = Line_Follower.Line_Follower()
lf.references = REFERENCES
bw = back_wheels.Back_Wheels()
fw = front_wheels.Front_Wheels()
bw.ready()
fw.ready()
fw.turning_max = 45

last_cmd_vel_time = None
timeout_duration = 1.0

def move_forward():
    bw.speed = forward_speed
    bw.forward()

def stop():
    bw.stop()

def detect_line():
    try:
        values = lf.read_analog()
        rospy.loginfo("Taking data from channel: {}".format(values.index(min(values))))
        if all(value > stop_threshold for value in values):
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
            if lowest_index == 0:
                fw.turn(60)
                move_forward()
            elif lowest_index == 1:
                fw.turn(75)
                move_forward()
            elif lowest_index == 2:
                move_forward()
            elif lowest_index == 3:
                fw.turn(105)
                move_forward()
            elif lowest_index == 4:
                fw.turn(120)
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
