#!/usr/bin/env python
import rospy
from sensor_msgs.msg import Image, CameraInfo
from cv_bridge import CvBridge, CvBridgeError
import cv2
import numpy as np


i = 0
collect = True
calibration = False
publishing = False
collected_images = []

bridge = CvBridge()

objpoints = [] # 3d point in real world space
imgpoints = [] # 2d points in image plane.



def callback(image_data):
    global bridge, collected_images, objpoints, imgpoints

    objp = np.zeros((6*7, 3), np.float32)
    objp[:,:2] = np.mgrid[0:7,0:6].T.reshape(-1,2) * 0.108
    criteria = (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 30, 0.001)
    error = 0.0

    img_data = bridge.imgmsg_to_cv2(image_data, desired_encoding="rgb8")

    if len(collected_images) < 37 and image_data not in collected_images:
        collected_images.append(image_data)
        gray = cv2.cvtColor(img_data,cv2.COLOR_BGR2GRAY)
        ret, corners = cv2.findChessboardCorners(gray, (7,6),None)
        if ret == True:
            objpoints.append(objp)
            corners2 = cv2.cornerSubPix(gray,corners,(11,11),(-1,-1),criteria)
            imgpoints.append(corners2)

            cv2.drawChessboardCorners(img_data, (7,6), corners2,ret)

    if len(collected_images) == 37:
        gray = cv2.cvtColor(img_data,cv2.COLOR_BGR2GRAY)
        ret, mtx, dist, rvecs, tvecs = cv2.calibrateCamera(objpoints, imgpoints, gray.shape[::-1], None, None)
        for i in range(len(objpoints) - 1):
            imgpointsKnown, _ = cv2.projectPoints(objpoints[i], rvecs[i], tvecs[i], mtx, dist)

            err = cv2.norm(imgpoints[i], imgpointsKnown, cv2.NORM_L2) / len(imgpointsKnown)
            error += err
    rospy.loginfo("Total error wil be shown when 37 images will be collected! (Images collected %s): %s", len(collected_images), error)

    camera = CameraInfo()
    camera.header.frame_id = "camera"
    camera.D = dist.flatten()
    camera.K = mtx.flatten()
    camera.R = np.eye(e).flatten()
    mtx1 = np.zeros((3, 4))
    mtx1[:,:-1] = mtx
    camera.P = mtx1.flatten()
    camera.height = image_data.height
    camera.width = image_data.width
    camera.distortion_model = 'plumb_bob'
    

def subscriber():

    rospy.init_node('camera_calibration', anonymous=True)

    rospy.Subscriber('/image_raw', Image, callback)

    rospy.spin()

if __name__ == "__main__":
    subscriber()