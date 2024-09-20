#!/usr/bin/env python3
import rospy
from std_msgs.msg import Float32
from SunFounder_Ultrasonic_Avoidance import Ultrasonic_Avoidance

class UltrasonicDist:
    def __init__(self):
        rospy.init_node('ultrasonic_dist', anonymous=True)
        
        # Initialize Publisher
        self.distance_pub = rospy.Publisher('ultrasonic_distance', Float32, queue_size=10)
        
        # Initialize Ultrasonic Sensor
        self.ua = Ultrasonic_Avoidance.Ultrasonic_Avoidance(20)
        self.rate = rospy.Rate(30)  

    def run(self):
        while not rospy.is_shutdown():
            # Read distance
            distance = self.ua.get_distance()
            
            if distance != -1:
                rospy.loginfo("Distance: %.2f cm", distance)
                self.distance_pub.publish(Float32(distance))
            else:
                rospy.logwarn("Error reading distance")
                self.distance_pub.publish(Float32(-1))  # Publish -1 for error
            
            self.rate.sleep()

if __name__ == '__main__':
    try:
        node = UltrasonicDist()
        node.run()
    except rospy.ROSInterruptException:
        rospy.loginfo("ROS Interrupt Exception. Stopping node...")
