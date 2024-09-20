#!/usr/bin/env python3
import rospy
from geometry_msgs.msg import Twist
from picar import back_wheels, front_wheels
import picar

picar.setup()

# Constants
FORWARD_SPEED = 50
TURNING_SPEED = 60
TURN_ANGLE = 30
MIN_SPEED = 45
MAX_SPEED = 50
ACCELERATION = 2  # Speed units per cycle

# Initialize Wheels and Motor
bw = back_wheels.Back_Wheels()
fw = front_wheels.Front_Wheels()
bw.ready()
fw.ready()
fw.turning_max = 45
fw.turn_straight()  # Ensure wheels are aligned at start

# Global variables
current_speed = 0
target_speed = 0

def scale_speed(input_speed):
    return max(MIN_SPEED, int(abs(input_speed) * (MAX_SPEED - MIN_SPEED) + MIN_SPEED))

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
    global current_speed, target_speed
    linear_vel = data.linear.x
    angular_vel = data.angular.z
    is_forward = linear_vel >= 0

    if abs(linear_vel) > 0.1:
        target_speed = scale_speed(linear_vel)
    else:
        target_speed = 0

    # Smooth acceleration while ensuring minimum speed
    if current_speed < target_speed:
        current_speed = min(max(current_speed + ACCELERATION, MIN_SPEED), target_speed)
    elif current_speed > target_speed:
        current_speed = max(current_speed - ACCELERATION, max(MIN_SPEED, target_speed))

    # Ensure minimum speed when moving
    if current_speed > 0:
        current_speed = max(current_speed, MIN_SPEED)
        if is_forward:
            move_forward(current_speed)
        else:
            move_backward(current_speed)
    else:
        stop()

    if abs(angular_vel) > 0.1:
        if angular_vel > 0:
            turn_left(TURN_ANGLE, max(current_speed, MIN_SPEED), is_forward)
        else:
            turn_right(TURN_ANGLE, max(current_speed, MIN_SPEED), is_forward)
    else:
        fw.turn_straight()

    rospy.loginfo(f"Linear: {linear_vel:.2f}, Angular: {angular_vel:.2f}, Speed: {current_speed}")

def teleop_motor_node():
    rospy.init_node('teleop_motor')
    rospy.Subscriber('cmd_vel', Twist, cmd_vel_callback)
    rospy.spin()

def shutdown_hook():
    stop()  # Ensure the back wheels are stopped
    fw.turn_straight()  # Ensure the front wheels are aligned
    rospy.loginfo("Stopping the motor and centering wheels...")

if __name__ == '__main__':
    try:
        rospy.on_shutdown(shutdown_hook)
        teleop_motor_node()
    except rospy.ROSInterruptException:
        rospy.loginfo("Ctrl+C detected. Stopping the node...")
    finally:
        shutdown_hook()
