#!/usr/bin/env python3
import rospy
from geometry_msgs.msg import Twist
from SunFounder_Line_Follower import Line_Follower
from picar import back_wheels
from picar import front_wheels
import picar
import time
import random

picar.setup()

forward_speed = 50
reverse_speed = 40  # Speed during reversing when off-track
turning_speed = 40  # Speed during turns
max_off_track_count = 5
backward_speed = 30
delay = 0.05  

# Line Follower Initialization
lf = Line_Follower.Line_Follower()

# Wheel Initialization
bw = back_wheels.Back_Wheels()
fw = front_wheels.Front_Wheels()
bw.ready()
fw.ready()
fw.turning_max = 45

last_cmd_vel_time = None
timeout_duration = 1.0

# State Management
state = "track_following"
track_lost_time = None
last_turn_direction = None  # "left" or "right"
off_track_count = 0
back_and_forth_count = 0  # Tracks number of back-and-forth movements

# Step values for turning angle calculation
a_step = 3
b_step = 10
c_step = 30
d_step = 45
turning_angle = 90

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
            return False, -1 
        
        # Find the index of lowest sensor value 
        lowest_index = values.index(min(values))
        return True, lowest_index

    except Exception as e:
        rospy.logerr("Error occurred: {}".format(e))
        stop()
        return False, -1

def cmd_vel_callback(data):
    global last_cmd_vel_time, state, track_lost_time, last_turn_direction, off_track_count, turning_angle, back_and_forth_count
    last_cmd_vel_time = rospy.get_time()
    
    line_detected, lowest_index = detect_line()
    
    if line_detected:
        if state == "track_following":
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
                    track_lost_time = rospy.get_time()
            else:
                stop()

        elif state == "recovery":
            # Recovery logic when the line is lost
            if lowest_index in [1, 2, 3]:  # Line recovered, switch back to track-following
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

    else:
        if state == "track_following":
            off_track_count += 1
            if off_track_count > max_off_track_count:
                state = "recovery"
                track_lost_time = rospy.get_time()
                if last_turn_direction == "left":
                    rospy.loginfo("Line lost. Reversing right.")
                    fw.turn(130)
                    move_backward()
                elif last_turn_direction == "right":
                    rospy.loginfo("Line lost. Reversing left.")
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



def attempt_random_reversals():
    # Attempt random left or right short reversal

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

def check_cmd_vel_timeout(event):
    global last_cmd_vel_time
    if last_cmd_vel_time is not None:
        current_time = rospy.get_time()
        elapsed_time = current_time - last_cmd_vel_time
        if elapsed_time > timeout_duration:
            stop()

def driver_motor_node():
    rospy.init_node('dmV7')
    rospy.Subscriber('cmd_vel', Twist, cmd_vel_callback)
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
