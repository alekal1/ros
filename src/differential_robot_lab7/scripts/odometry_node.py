#!/usr/bin/env python

import rospy
import math
import tf
from differential_robot.msg import counter_message
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Point, Pose, Quaternion, Twist, Vector3

class OdometryNode():

    def __init__(self):
        rospy.loginfo('Initializing odometry node')

        ## Class constants:
        self.wheel_diam = 70.0/1000.0   # Wheels diameter [m]
        self.wheel_dist = 0.165         # Wheels distance [m]
        self.cpr = 1440.0               # Encoders clicks per revolution
        self.assume_folding = 500.0     # If absolute difference between last and current encoder pos is greater than this,
                                        # we assume that the encoder has passed 0-point

        ## Class variables:
        self.init = True         # First message not received yet
        self.currentTime = None
        self.lastReceive = 0.0   # Time of previous message
        self.currReceive = 0.0   # Time of current message
        self.diffTime = 0.0      # Time difference between current and last receive
        self.lastPosL = 0        # Left wheel last position
        self.lastPosR = 0        # Right wheel last position
        self.currPosL = 0        # Left wheel current position
        self.currPosR = 0        # Right wheel current position
        self.rotSpeedL = 0.0     # Left wheel speed [rad/s]
        self.rotSpeedR = 0.0     # Right wheel speed [rad/s]
        self.linSpeedL = 0.0     # Left wheel speed [m/s]
        self.linSpeedR = 0.0     # Right wheel speed [m/s]
        self.fwdSpeedC = 0.0     # Center forward speed [m/s]
        self.rotSpeed = 0.0      # Rate of rotation of the robot [rad/s]
        self.rotAngle = 0.0      # Angle of rotation of robot [rad]
        self.posX = 0.0
        self.posY = 0.0

        self.odom = Odometry()
        self.odom.header.frame_id = "odom"
        self.odom.child_frame_id = "base_link"

    def callback(self,data):
        if data.count_left >= self.cpr or data.count_right >= self.cpr:
            return               # Sanity check: We sometimes receive illegal encoder values
        self.lastReceive = self.currReceive
        self.currentTime = rospy.Time.now()
        self.currReceive = self.currentTime.to_sec()
        self.diffTime = self.currReceive - self.lastReceive
        if self.diffTime == 0:
            return
        self.lastPosL = self.currPosL
        self.lastPosR = self.currPosR
        self.currPosL = data.count_left
        self.currPosR = data.count_right
        rospy.loginfo(self.currReceive)
        rospy.loginfo('Time delta: %f',self.diffTime)
        rospy.loginfo('-------------------------------')
        rospy.loginfo('Pos L: %d', self.currPosL)
        rospy.loginfo('Pos R: %d', self.currPosR)
        rospy.loginfo('-------------------------------')
        if self.init == True:
            self.init = False
            rospy.loginfo('\n')
            return
        self.calculate_wheel_speeds()
        self.calculate_odometry()
        rospy.loginfo('\n')
        

    def calculate_odometry(self):
        # https://gist.github.com/atotto/f2754f75bedb6ea56e3e0264ec405dcf
        self.odom.header.stamp = self.currentTime
        self.fwdSpeedC = (self.linSpeedL + self.linSpeedR) / 2.0
        self.rotSpeed = (self.linSpeedR - self.linSpeedL) / self.wheel_dist
        
        # get increments in position and yaw from velocities
        delta_x = (self.fwdSpeedC * math.cos(self.rotAngle)) * self.diffTime
        delta_y = (self.fwdSpeedC * math.sin(self.rotAngle)) * self.diffTime
        delta_th = self.rotSpeed * self.diffTime

        # sum up with previous values to complete integral calculations
        self.posX += delta_x
        self.posY += delta_y
        self.rotAngle += delta_th

        # since all odometry is 6DOF we'll need a quaternion created from yaw
        odom_quat = tf.transformations.quaternion_from_euler(0, 0, self.rotAngle)
        
        # set the position and velocity
        self.odom.pose.pose = Pose(Point(self.posX, self.posY, 0.), Quaternion(*odom_quat))
        self.odom.twist.twist = Twist(Vector3(self.fwdSpeedC, 0, 0), Vector3(0, 0, self.rotSpeed))

        self.odom_pub.publish(self.odom)
    

    def calculate_wheel_speeds(self):
        
        diffL = self.currPosL - self.lastPosL
        diffR = self.currPosR - self.lastPosR

        if abs(diffL) <= self.assume_folding:
            self.rotSpeedL = (diffL / self.cpr) * 2 * math.pi / self.diffTime
            self.linSpeedL = math.pi * self.wheel_diam * (diffL / self.cpr) / self.diffTime
        else:
            if self.lastPosL > self.currPosL:
                self.rotSpeedL = ((self.currPosL + self.cpr - self.lastPosL) / self.cpr * 2 * math.pi) / self.diffTime
                self.linSpeedL = math.pi * self.wheel_diam * ((self.currPosL + self.cpr - self.lastPosL) / self.cpr) / self.diffTime
            else:
                self.rotSpeedL = -((self.lastPosL + self.cpr - self.currPosL) / self.cpr * 2 * math.pi) / self.diffTime
                self.linSpeedL = - math.pi * self.wheel_diam * ((self.lastPosL + self.cpr - self.currPosL) / self.cpr) / self.diffTime

        if abs(diffR) <= self.assume_folding:
            self.rotSpeedR = (diffR / self.cpr) * 2 * math.pi / self.diffTime
            self.linSpeedR = math.pi * self.wheel_diam * (diffR / self.cpr) / self.diffTime
        else:
            if self.lastPosR > self.currPosR:
                self.rotSpeedR = ((self.currPosR + self.cpr - self.lastPosR) / self.cpr * 2 * math.pi) / self.diffTime
                self.linSpeedR = math.pi * self.wheel_diam * ((self.currPosR + self.cpr - self.lastPosR) / self.cpr) / self.diffTime
            else:
                self.rotSpeedR = -((self.lastPosR + self.cpr - self.currPosR) / self.cpr * 2 * math.pi) / self.diffTime
                self.linSpeedR = -math.pi * self.wheel_diam * ((self.lastPosR + self.cpr - self.currPosR) / self.cpr) / self.diffTime
        
        rospy.loginfo('Rot L: %f', self.rotSpeedL)
        rospy.loginfo('Rot R: %f', self.rotSpeedR)
        rospy.loginfo('-------------------------------')

        rospy.loginfo('Lin L: %f', self.linSpeedL)
        rospy.loginfo('Lin R: %f', self.linSpeedR)
        rospy.loginfo('-------------------------------')
        
    def run(self):
        """ Main class method."""
        rospy.Subscriber('encoders_output', counter_message, self.callback)
        self.odom_pub = rospy.Publisher("odom", Odometry, queue_size=50)
        # spin() simply keeps python from exiting until this node is stopped
        rospy.spin()


if __name__ == '__main__':
    rospy.init_node('odometry', anonymous=True)
    odometry = OdometryNode()
    odometry.run()