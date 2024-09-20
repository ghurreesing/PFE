#!/usr/bin/env python3

import rospy
import cv2
from sensor_msgs.msg import Image
from cv_bridge import CvBridge

def camera_publisher():

    rospy.init_node('camera_publisher', anonymous=True)

    pub = rospy.Publisher('image_raw', Image, queue_size=10)
    rate = rospy.Rate(30)

    bridge = CvBridge()

    cap = cv2.VideoCapture('/dev/video0', cv2.CAP_V4L)
    # Set dimensions
    cap.set(cv2.CAP_PROP_FRAME_WIDTH, 640)
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)
    while not rospy.is_shutdown():
        # Take frame
        ret, frame = cap.read()
        if ret:

            frame = cv2.rotate(frame, cv2.ROTATE_180)

            ros_image = bridge.cv2_to_imgmsg(frame, "bgr8")
            # Publish image
            pub.publish(ros_image)
        rate.sleep()

    cap.release()

if __name__ == '__main__':
    try:
        camera_publisher()
    except rospy.ROSInterruptException:
        pass
