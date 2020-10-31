#!/usr/bin/env python
import roslib
import rospy
import rospkg
import numpy as np
import glob
import cv2
import random as rng
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError


class VideoSubscriber:

    def __init__(self):
        self.video_subscriber = rospy.Subscriber(
            '/video_stream', Image, self.callback)
        self.frame_publisher = rospy.Publisher(
            '/processed_video', Image, queue_size=1000)
        self.bridge = CvBridge()
        self.min_value = np.array(
            [20, 160, 160], dtype="uint8")  # Minimum color range
        self.max_value = np.array(
            [25, 255, 255], dtype="uint8")  # Maximum color range

    def callback(self, video_frame_data):
        frame_img = self.bridge.imgmsg_to_cv2(
            video_frame_data, desired_encoding="passthrough")
        frame_hsv = cv2.cvtColor(frame_img, cv2.COLOR_BGR2HSV)

        duck_mask = cv2.inRange(
            frame_hsv, self.min_value, self.max_value)  # ???

        self.thresh_callback(frame_hsv, duck_mask)

    def thresh_callback(self, frame_hsv, canny_output):
        # rospy.loginfo("HERE!")
        # canny_output = cv2.Canny(frame_hsv, self.thresh, self.thresh * 2)

        # Find contours
        _, contours, _ = cv2.findContours(
            canny_output, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)

        contours_poly = [None]*len(contours)
        boundRect = [None]*len(contours)
        centers = [None]*len(contours)
        radius = [None]*len(contours)

        rospy.loginfo(len(contours))
        for i, c in enumerate(contours):
            contours_poly[i] = cv2.approxPolyDP(c, 3, True)
            boundRect[i] = cv2.boundingRect(contours_poly[i])
            centers[i], radius[i] = cv2.minEnclosingCircle(contours_poly[i])

        drawing = np.zeros(
            (canny_output.shape[0], canny_output.shape[1], 3), dtype=np.uint8)

        # Draw contours
        for i in range(len(contours)):
            color = (255, 255, 0)  # Yellow color in RGB
            cv2.drawContours(drawing, contours, i, color)
            cv2.rectangle(drawing, (int(boundRect[i][0]), int(boundRect[i][1])),
                          (int(boundRect[i][0]+boundRect[i][2]), int(boundRect[i][1]+boundRect[i][3])), color, 2)
            cv2.circle(drawing, (int(centers[i][0]), int(
                centers[i][1])), int(radius[i]), color, 2)

        msg = self.bridge.cv2_to_imgmsg(drawing, encoding='passthrough')
        self.frame_publisher.publish(msg)

    def run(self):
        while not rospy.is_shutdown():
            rospy.spin()


if __name__ == "__main__":
    rospy.init_node('object_recognition')
    sub = VideoSubscriber()
    sub.run()
