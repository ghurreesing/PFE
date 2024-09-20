#!/usr/bin/env python3

import rospy
import cv2
from cv_bridge import CvBridge
from sensor_msgs.msg import Image
from pyzbar import pyzbar
import numpy as np

class QRDetector:
    def __init__(self):
        self.bridge = CvBridge()
        self.image_sub = rospy.Subscriber("image_raw", Image, self.image_callback)
        self.image_pub = rospy.Publisher("image_with_qr", Image, queue_size=10)

    def image_callback(self, data):
        try:
            cv_image = self.bridge.imgmsg_to_cv2(data, "bgr8")
        except CvBridgeError as e:
            rospy.logerr(e)
            return

        # Convert image to grayscale
        gray = cv2.cvtColor(cv_image, cv2.COLOR_BGR2GRAY)

        # Detect QR codes
        qr_codes = pyzbar.decode(gray)

        for qr in qr_codes:
            # Extract QR code data
            qr_data = qr.data.decode('utf-8')
            rospy.loginfo(f"QR Code detected: {qr_data}")

            # Get QR code coordinates
            points = qr.polygon

            # If it's not a polygon with 4 points, find the bounding rect
            if len(points) != 4:
                points = qr.rect

            # Construct a numpy array of the points
            pts = np.array(points, dtype=int)
            pts = pts.reshape((-1, 1, 2))

            # Draw green polygon around the QR code
            cv2.polylines(cv_image, [pts], True, (0, 255, 0), 2)

            # Put the QR code data on the image
            cv2.putText(cv_image, qr_data, (qr.rect.left, qr.rect.top - 10),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2)

        # Publish the image with QR code overlay
        try:
            self.image_pub.publish(self.bridge.cv2_to_imgmsg(cv_image, "bgr8"))
        except CvBridgeError as e:
            rospy.logerr(e)

def main():
    rospy.init_node('qr_detector', anonymous=True)
    QRDetector()
    try:
        rospy.spin()
    except KeyboardInterrupt:
        rospy.loginfo("Shutting down QR Detector node")

if __name__ == '__main__':
    main()
