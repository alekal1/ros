#!/usr/bin/env python
import roslib
import rospy
import rospkg
import numpy as np
import glob
import cv2
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError

class VideoSubscriber:

    def __init__(self):
        self.video_subscriber = rospy.Subscriber('/video_stream', Image, self.callback)
        self.bridge = CvBridge()
        self.min_value = np.array([20, 160, 160], dtype="uint8")
        self.max_value = np.array([25, 255, 255], dtype="uint8")
    

    def callback(self, video_frame_data):
        frame_img = self.bridge.imgmsg_to_cv2(video_frame_data, desired_encoding="passthrough")
        hsv = cv2.cvtColor(frame_img, cv2.COLOR_BGR2HSV)

        duck_mask = cv2.inRange(hsv, self.min_value, self.max_value)
        res = cv2.bitwise_and(frame_img, frame_img, mask=duck_mask)


    def run(self):
        while not rospy.is_shutdown():
            rospy.spin()

if __name__ == "__main__":
    rospy.init_node('object_recognition')
    sub = VideoSubscriber()
    sub.run()