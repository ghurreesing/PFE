#!/usr/bin/env python3
import rospy
from std_msgs.msg import Float32
from picar import front_wheels, back_wheels
import picar
import signal
import random
import time

class ObstacleAvoidance:
    def __init__(self):
        rospy.init_node('obstacle_avoidance', anonymous=True)

        # Initialize Subscriber
        rospy.Subscriber('ultrasonic_distance', Float32, self.distance_callback)

        # Initialize Motor Controllers
        self.fw = front_wheels.Front_Wheels(db='config')
        self.bw = back_wheels.Back_Wheels(db='config')
        self.fw.turning_max = rospy.get_param('~turning_max', 45)

        # Constants and Parameters
        self.threshold = rospy.get_param('~threshold', 16) 
        self.forward_speed = rospy.get_param('~forward_speed', 50)
        self.turn_speed = rospy.get_param('~turn_speed', 60)
        self.backward_speed = rospy.get_param('~backward_speed', 40)
        self.rate = rospy.Rate(30) 

        # Internal State
        self.shutdown_flag = False
        self.distance = None
        self.last_turn_direction = None

        # Set up signal handler
        signal.signal(signal.SIGINT, self.signal_handler)
        rospy.on_shutdown(self.shutdown_hook)

    def signal_handler(self, sig, frame):
        rospy.loginfo("Signal handler called with signal: %d", sig)
        self.shutdown_flag = True

    def distance_callback(self, data):
        self.distance = data.data
        rospy.loginfo(f"Received distance: {self.distance:.2f} cm")

    def move_forward(self):
        self.fw.turn_straight()
        self.bw.forward()
        self.bw.speed = self.forward_speed
        rospy.loginfo("Moving forward")

    def stop(self):
        self.bw.stop()
        rospy.loginfo("Stopping")

    def turn(self, direction):
        angle = 90 + (self.fw.turning_max if direction == 'right' else -self.fw.turning_max)
        self.fw.turn(angle)
        self.bw.forward()
        self.bw.speed = self.turn_speed
        rospy.loginfo(f"Turning {direction}")

    def back_up(self, duration=1.0):
        self.bw.backward()
        self.bw.speed = self.backward_speed
        rospy.loginfo("Backing up")
        time.sleep(duration)
        self.bw.stop()

    def run(self):
        while not rospy.is_shutdown() and not self.shutdown_flag:
            if self.distance is not None:
                if self.distance < self.threshold:
                    rospy.loginfo("Obstacle detected!")
                    self.stop()
                    self.back_up(1)  # Reverse
                    direction = self.randomized_turn_direction()
                    self.turn(direction)
                    time.sleep(1)  # Turn
                    self.stop()
                    time.sleep(0.2)  # Stop

                # If no obstacle detected, continue moving forward
                self.move_forward()
            else:
                rospy.logwarn("No distance data received yet.")

            self.rate.sleep()

        rospy.loginfo("Obstacle Avoidance node stopped.")

    def randomized_turn_direction(self):
        if self.last_turn_direction is None or self.last_turn_direction == 'right':
            direction = 'left'
        else:
            direction = 'right'

        if random.choice([True, False]):
            direction = 'right' if direction == 'left' else 'left'

        self.last_turn_direction = direction
        return direction

    def shutdown_hook(self):
        self.shutdown_flag = True
        self.bw.stop()
        rospy.loginfo("Shutdown complete.")

if __name__ == '__main__':
    try:
        node = ObstacleAvoidance()
        node.run()
    except rospy.ROSInterruptException:
        rospy.loginfo("ROS Interrupt Exception. Stopping node...")
        node.shutdown_hook()
    except Exception as e:
        rospy.logerr("Exception occurred: %s", e)
        node.shutdown_hook()
