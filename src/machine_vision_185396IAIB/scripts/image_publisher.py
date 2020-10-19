#!/usr/bin/env python

import roslib
import rospy
import rospkg
import numpy as np
import glob
import cv2
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError



class Publisher():
    
    def __init__(self, path):
        self.images = glob.glob(path+'/data/images/*.png') # Loads the images
        self.publisher = rospy.Publisher('/image_raw', Image, queue_size=1000) # Init publisher
        self.rate = rospy.Rate(2)

        self.len_images = len(self.images)
        self.bridge = CvBridge()
        self.current_index = 0
        self.msg = Image()

    def create_image_message(self, data):
        img = cv2.imread(data)
        self.msg = self.bridge.cv2_to_imgmsg(img, encoding="rgb8")
        self.msg.header.frame_id = "camera"

    def publish(self):
        if self.current_index % self.len_images == 0:
            self.current_index = 0
        self.create_image_message(self.images[self.current_index])
        self.publisher.publish(self.msg)
        self.current_index += 1

    
    def run(self):
        while not rospy.is_shutdown():
            self.publish()
            self.rate.sleep()


if __name__ == '__main__':
    rospy.init_node('image_publisher')

    rospack = rospkg.RosPack()
    path = rospack.get_path('machine_vision_185396IAIB')

    publisher = Publisher(path)
    publisher.run()
