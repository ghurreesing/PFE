#!/usr/bin/env python3
import rospy
from geometry_msgs.msg import Twist
from std_msgs.msg import Float32
from SunFounder_Line_Follower import Line_Follower
from picar import back_wheels, front_wheels
import picar
import time
import random

picar.setup()

# Constants
forward_speed = 50
reverse_speed = 40  # Speed during reversing when off-track
turning_speed = 40  # Speed during turns
max_off_track_count = 5
backward_speed = 30
delay = 0.05  # Time delay in seconds
obstacle_distance_threshold = 15.0  # cm

# Line Follower Initialization
lf = Line_Follower.Line_Follower()

# Wheel Initialization
bw = back_wheels.Back_Wheels()
fw = front_wheels.Front_Wheels()
bw.ready()
fw.ready()
fw.turning_max = 45

# State Management
state = "track_following"
turn_direction = 0  # 1 for left-first, 0 for right-first
off_track_count = 0
back_and_forth_count = 0  # Tracks the number of back-and-forth movements
obstacle_detected = False

# Step values for turning angle calculation
a_step = 3
b_step = 10
c_step = 30
d_step = 45
turning_angle = 90
last_turn_direction = None  # "left" or "right"

def move_forward():
    bw.speed = forward_speed
    bw.forward()

def move_backward():
    bw.speed = reverse_speed
    bw.backward()

def stop():
    bw.stop()

def detect_line():
    try:
        values = lf.read_analog()
        rospy.loginfo("Sensor values: {}".format(values))
        
        # Calculate dynamic threshold
        avg_value = sum(values) / len(values)
        dynamic_threshold = avg_value * 0.7
        
        # Determine if the line is detected
        if all(value > dynamic_threshold for value in values):
            return False, -1  # No line detected
        
        # Find the index of the lowest sensor value (indicating the line)
        lowest_index = values.index(min(values))
        return True, lowest_index

    except Exception as e:
        rospy.logerr("Error occurred: {}".format(e))
        stop()
        return False, -1

def ultrasonic_callback(data):
    global obstacle_detected, state
    if data.data > 0 and data.data < obstacle_distance_threshold:
        obstacle_detected = True
        state = "obstacle_avoidance"
    else:
        obstacle_detected = False

def cmd_vel_callback(data):
    global state, off_track_count, turn_direction, back_and_forth_count, last_turn_direction, turning_angle

    if state == "obstacle_avoidance":
        rospy.loginfo("Obstacle detected! Engaging obstacle avoidance...")
        avoid_obstacle()

    elif state == "track_following":
        line_detected, lowest_index = detect_line()
        
        if line_detected:
            # Primary logic for line following based on sensor index
            if lowest_index == 2:
                step = 0
                off_track_count = 0
                back_and_forth_count = 0  # Reset counter when moving straight
                fw.turn(90)
                move_forward()
                last_turn_direction = None
            elif lowest_index == 1 or lowest_index == 3:
                step = a_step
                off_track_count = 0
                back_and_forth_count = 0  # Reset counter when turning normally
                turning_angle = 90 - step if lowest_index == 1 else 90 + step
                fw.turn(turning_angle)
                move_forward()
                last_turn_direction = "left" if lowest_index == 1 else "right"
            elif lowest_index == 0 or lowest_index == 4:
                step = c_step
                off_track_count = 0
                back_and_forth_count = 0  # Reset counter when turning significantly
                turning_angle = 90 - step if lowest_index == 0 else 90 + step
                fw.turn(turning_angle)
                move_forward()
                last_turn_direction = "left" if lowest_index == 0 else "right"
            elif lowest_index == -1:
                off_track_count += 1
                if off_track_count > max_off_track_count:
                    state = "recovery"
                    rospy.loginfo("Switching to recovery mode due to line loss.")
            else:
                stop()
        else:
            off_track_count += 1
            if off_track_count > max_off_track_count:
                state = "recovery"
                rospy.loginfo("Line lost, switching to recovery mode.")

    elif state == "recovery":
        # Recovery logic when the line is lost
        line_detected, lowest_index = detect_line()
        if lowest_index in [1, 2, 3]: 
            rospy.loginfo("Line recovered, switching back to track_following state.")
            state = "track_following"
            off_track_count = 0
            move_forward()
            back_and_forth_count = 0  # Reset counter when line is recovered
        else:
            # If the line is still lost, continue reversing until the line is found
            if last_turn_direction == "left":
                fw.turn(130)
                move_backward()
            elif last_turn_direction == "right":
                fw.turn(50)
                move_backward()
            else:
                rospy.loginfo("Line lost. No last turn direction known. Reversing until line is found.")
                move_backward()
                if not line_detected:
                    line_detected, _ = detect_line()
                rospy.loginfo("Line found after reversing. Moving forward.")
                move_forward()
                back_and_forth_count += 1
                if back_and_forth_count >= 3:
                    attempt_random_reversals()
                    back_and_forth_count = 0
                state = "track_following"

def avoid_obstacle():
    global state, turn_direction, last_turn_direction

    stop()

    if turn_direction == 0:  # Right-first avoidance
        rospy.loginfo("Avoiding obstacle by turning right first.")
        fw.turn(50)
        move_backward()
        rospy.sleep(0.4)
        stop()
        rospy.loginfo("Turning left to regain the line.")
        fw.turn(130)
        move_forward()
    else:  # Left-first avoidance
        rospy.loginfo("Avoiding obstacle by turning left first.")
        fw.turn(130)
        move_backward()
        rospy.sleep(0.4)
        stop()
        rospy.loginfo("Turning right to regain the line.")
        fw.turn(50)
        move_forward()

    rospy.sleep(0.7)  # Wait for movement

    while True:
        line_detected, _ = detect_line()
        if line_detected:
            rospy.loginfo("Line detected after obstacle avoidance. Resuming track following.")
            state = "track_following"
            break
        else:
            if turn_direction == 0:
                fw.turn(50)
            else:
                fw.turn(130)
            rospy.sleep(0.1)  # Small sleep to allow the robot to turn and search for the line

def attempt_random_reversals():
    """
    Attempt a random left or right short reversal when the robot is stuck.
    """
    direction = random.choice(["left", "right"])
    rospy.loginfo(f"Attempting {direction} short reversal.")
    
    if direction == "left":
        fw.turn(130)
        move_backward()
        rospy.sleep(0.5)
        stop()
    elif direction == "right":
        fw.turn(50)
        move_backward()
        rospy.sleep(0.5)
        stop()

    # After the first attempt, check if line is found, if not try the opposite
    line_detected, _ = detect_line()
    if not line_detected:
        rospy.loginfo(f"Line not found after {direction} reversal. Trying opposite direction.")
        if direction == "left":
            fw.turn(50)
        else:
            fw.turn(130)
        move_backward()
        rospy.sleep(0.3)
        stop()
    line_detected, _ = detect_line()

def driver_motor_node():
    rospy.init_node('Line_with_Avoidance')

    # Subscribe to the cmd_vel and ultrasonic_distance topics
    rospy.Subscriber('cmd_vel', Twist, cmd_vel_callback)
    rospy.Subscriber('ultrasonic_distance', Float32, ultrasonic_callback)
    
    rospy.spin()

def shutdown_hook():
    stop()
    fw.turn_straight()
    rospy.loginfo("Stopping the motor...")

if __name__ == '__main__':
    try:
        rospy.on_shutdown(shutdown_hook)
        driver_motor_node()
    except rospy.ROSInterruptException:
        rospy.loginfo("Ctrl+C detected. Stopping the node...")
        stop()
