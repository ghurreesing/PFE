#!/usr/bin/env python3
import rospy
from geometry_msgs.msg import Twist
from picar import back_wheels, front_wheels
import picar

picar.setup()

# Constants
forward_speed = 50
turning_speed = 60

# Wheel Initialization
bw = back_wheels.Back_Wheels()
fw = front_wheels.Front_Wheels()
bw.ready()
fw.ready()
fw.turning_max = 45

def move_forward(speed):
    bw.speed = speed
    bw.forward()

def move_backward(speed):
    bw.speed = speed
    bw.backward()

def stop():
    bw.stop()

def turn_left(angle, speed, is_forward):
    fw.turn(90 - angle)
    bw.speed = speed
    if is_forward:
        bw.forward()
    else:
        bw.backward()

def turn_right(angle, speed, is_forward):
    fw.turn(90 + angle)
    bw.speed = speed
    if is_forward:
        bw.forward()
    else:
        bw.backward()

def cmd_vel_callback(data):
    linear_vel = data.linear.x
    angular_vel = data.angular.z
    is_forward = linear_vel >= 0

    if linear_vel > 0:
        move_forward(forward_speed)
    elif linear_vel < 0:
        move_backward(forward_speed)
    else:
        stop()

    if angular_vel > 0:
        turn_left(30, turning_speed, is_forward)
    elif angular_vel < 0:
        turn_right(30, turning_speed, is_forward)
    else:
        fw.turn_straight()

def teleop_motor_node():
    rospy.init_node('teleop_motor')
    rospy.Subscriber('cmd_vel', Twist, cmd_vel_callback)
    rospy.spin()

def shutdown_hook():
    stop()
    rospy.loginfo("Stopping the motor...")

if __name__ == '__main__':
    try:
        rospy.on_shutdown(shutdown_hook)
        teleop_motor_node()
    except rospy.ROSInterruptException:
        rospy.loginfo("Ctrl+C detected. Stopping the node...")
        stop()
