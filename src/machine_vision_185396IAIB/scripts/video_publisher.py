#!/usr/bin/env python
import roslib
import rospy
import rospkg
import numpy as np
import glob
import cv2
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError

class VideoPublisher:
     
    def __init__(self, path):
        self.video = glob.glob(path+'/data/videos/*.mp4')
        self.publisher = rospy.Publisher('/video_stream', Image, queue_size=100)
        self.bridge = CvBridge() 
        self.frame_img = Image()
        self.cap = cv2.VideoCapture(self.video[0])

        self.rate = rospy.Rate(self.cap.get(cv2.CAP_PROP_FPS))
        self.video_frames = self.cap.get(cv2.CAP_PROP_FRAME_COUNT) # Get total video frames  [NB! cap.get(cv2.cv.CV_CAP_PROP_FRAME_COUNT) is not working so it's harcoded]
        self.frame_counter = 0
    
    def create_frame_image(self, frame):
        self.frame_img = self.bridge.cv2_to_imgmsg(frame, encoding="passthrough")


    def publish_video_frame(self):
        while (self.cap.isOpened()):
            ret, frame = self.cap.read()
            self.frame_counter += 1
            if self.frame_counter == self.video_frames:
                self.frame_counter = 0
                self.cap = cv2.VideoCapture(self.video[0]) # Recaptures video when it's finished
            if ret == True:
                self.create_frame_image(frame)
                self.publisher.publish(self.frame_img)
                if cv2.waitKey(25) & 0xFF == ord('q'):
                    break
        self.cap.release()


    def run(self):
        while not rospy.is_shutdown():
            self.publish_video_frame()
            self.rate.sleep()


if __name__ == "__main__":
    rospy.init_node('video_publisher')

    rospack = rospkg.RosPack()
    path = rospack.get_path('machine_vision_185396IAIB')

    publisher = VideoPublisher(path)
    publisher.run()
