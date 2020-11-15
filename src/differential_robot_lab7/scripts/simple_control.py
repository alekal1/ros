#!/usr/bin/env python

"""
Solution to home assignment 7 (Robot Control). Node to take a set of
waypoints and to drive a differential drive robot through those waypoints
using a simple PD controller and provided odometry data.

@author: 
@date: 
@input: Odometry as nav_msgs Odometry message
@output: body velocity commands as geometry_msgs Twist message.
"""

import math
import rospy
import numpy as np
from geometry_msgs.msg import Twist, Point
from visualization_msgs.msg import MarkerArray, Marker
from nav_msgs.msg import Odometry
from tf.transformations import euler_from_quaternion


class PDController:
    def __init__(self):
        self.Kd = rospy.get_param("/controller_waypoints/controller/Kd")
        self.Kp = rospy.get_param("/controller_waypoints/controller/Kp")
        self.distance_margin = rospy.get_param("/controller_waypoints/mission/distance_margin")
        self.waypoints = rospy.get_param("/controller_waypoints/mission/waypoints")
        self.wpIndex = 0
        rospy.loginfo("Got static data from parameter server:")
        rospy.loginfo(" Mission parameters:")
        rospy.loginfo("  Distance margin: %.2f",self.distance_margin)
        rospy.loginfo("  Waypoints (#: x|y):")
        wp = 1
        for waypoint in self.waypoints:
            rospy.loginfo("   %d: %.1f|%.1f",wp,waypoint[0],waypoint[1])
            wp += 1
        rospy.loginfo(" Controller parameters:")
        rospy.loginfo("  Proportional gains: %.2f, %.2f",self.Kp[0],self.Kp[1])
        rospy.loginfo("  Derivative gains  : %.2f, %.2f",self.Kd[0],self.Kd[1])
        rospy.loginfo("----------------------------------------------------------")
        self.waypoint_pub = None
        self.odometry = None      # stores data from odometry message; Odometry
        self.position = Point()   # current position of robot;         Point (3D vector: .x, .y, .z)
        self.heading = 0.0        # current orientation of robot (rad yaw)
        self.errors = [0.0,0.0]   # 2D vector: current errors in distance and heading
        self.de = [0.0,0.0]       # 2D vector: change of errors in distance and heading
        self.dt = 0               # time difference between current and last error
        self.rxTime = 0.0         # time of reception of last odom message (float seconds)
        self.done = False
        self.init = True
        self.vel_cmd = [0.0,0.0]  # calculated velocities (linear, angular)
        self.publisher_cmd_vel = rospy.Publisher("/controller_diffdrive/cmd_vel",Twist,queue_size=10)
        self.publisher_waypoints = rospy.Publisher("/mission_control/waypoints",MarkerArray,queue_size=10)
        self.marker_array = None
        self.twist = None


    def wrapAngle(self, angle):
        """
        helper function that returns angle wrapped between +- Pi
        @param: self
        @param: angle - angle to be wrapped in [rad]
        @result: returns wrapped angle -Pi <= angle <= Pi
        """        
        if angle > math.pi or angle < -1*math.pi:
            return -(2*math.pi - angle)

    def rad2deg(self, angle):
        """
        helper function to transform an angle from radians to degrees
        @param: self
        @param: angle - angle in [rad]
        @result: returns angle in [deg]
        """
        angle = 180.0 * angle / math.pi
        return angle

    def run(self):
        """
        Main loop of class.
        @param: self
        @result: runs the step function for controller update
        """

        rospy.Subscriber('odom', Odometry, self.onOdom)
        self.waypoint_pub = rospy.Publisher("waypoints", Marker, queue_size=50)
        # spin() simply keeps python from exiting until this node is stopped
        rospy.loginfo("Waiting for odom message...")
        rospy.spin()    
        

    def setNextWaypoint(self):
        """
        Removes current waypoint from list and sets next one as the current target
        @param: self
        @result: returns True if the next waypoint exists and has been set, otherwise False
        """        
        if not self.waypoints:
            return False
        self.waypoints.pop(0)
        if not self.waypoints:
            return False
        self.wpIndex += 1
        rospy.loginfo("----------------------------------------------")
        rospy.loginfo("                Next waypoint                 ")
        rospy.loginfo("----------------------------------------------")
        return True

    def isWaypointReached(self):
        """
        Checks if waypoint is reached based on user-defined threshold
        @param: self
        @result: returns True if waypoint is reached, otherwise False
        """
        if not self.waypoints:
            return False
        if math.sqrt((self.position.x - self.waypoints[0][0]) ** 2 + (self.position.y - self.waypoints[0][1]) ** 2) < self.distance_margin:
            return True        
        return False

    def controller(self):
        """
        Takes the errors and calculates velocities from it, according to control algorithm specs
        @param: self
        @result: sets the values in self.vel_cmd
        """
        if self.done:
            self.vel_cmd = [0.0,0.0]
            return

        # Determine dt
        now = rospy.Time.now().to_sec()
        self.dt = now - self.rxTime
        self.rxTime = now
        if self.init:
            self.init = False
            return
        if self.dt == 0:
            return

        # Determine current errors and their differentials in distance and yaw
        rospy.loginfo("Current position:")
        rospy.loginfo(" x: %.1f",self.position.x)
        rospy.loginfo(" y: %.1f",self.position.y)
        lastErrors = [self.errors[0],self.errors[1]]
        rospy.loginfo("Current waypoint: %d",self.wpIndex)
        rospy.loginfo(" Waypoint x: %.1f",self.waypoints[0][0])
        rospy.loginfo(" Waypoint y: %.1f",self.waypoints[0][1])
        self.errors[0] = np.sqrt( (self.waypoints[0][0]-self.position.x)**2 + (self.waypoints[0][1]-self.position.y)**2 )
        diff_y = self.waypoints[0][1]-self.position.y
        diff_x = self.waypoints[0][0]-self.position.x
        # if diff_x != 0:
        #     wp_heading = math.atan( diff_y / diff_x )
        # else:
        #     wp_heading = math.pi/2.0
        wp_heading = math.atan2( diff_y , diff_x )
        self.errors[1] = wp_heading - self.heading
        self.wrapAngle(self.errors[1])
        self.de[0] = self.errors[0] - lastErrors[0]
        self.de[1] = self.errors[1] - lastErrors[1]

        # Print the errors and their differentials
        rospy.loginfo("Current errors:")
        rospy.loginfo(" DeltaX [m]   : %.2f",diff_x)
        rospy.loginfo(" DeltaY [m]   : %.2f",diff_y)
        rospy.loginfo(" Distance [m] : %.2f",self.errors[0])
        rospy.loginfo(" Heading [rad]: %.2f",self.errors[1])
        rospy.loginfo(" Heading [deg]: %.2f",self.rad2deg(self.errors[1]))
        # rospy.loginfo("Differential of errors:")
        # rospy.loginfo(" Distance [m] : %.4f",self.de[0])
        # rospy.loginfo(" Heading [rad]: %.4f",self.de[1])
        # rospy.loginfo(" Heading [deg]: %.4f",self.rad2deg(self.de[1]))
        # rospy.loginfo("Time difference [s]: %.4f",self.dt)
        # rospy.loginfo("")

        # Calculate command velocities
        lin_p = self.Kp[0] * self.errors[0]
        lin_d = self.Kd[0] * (self.de[0]/self.dt)
        ang_p = self.Kp[1] * self.errors[1]
        ang_d = self.Kd[1] * (self.de[1]/self.dt)
        linear = lin_p + lin_d
        angular = ang_p + ang_d
        self.vel_cmd = [linear , angular]
        rospy.loginfo("PD outputs:")
        rospy.loginfo(" Linear  [P | D]: %.2f | %.2f",lin_p,lin_d)
        rospy.loginfo(" Angular [P | D]: %.2f | %.2f",ang_p,ang_d)

        rospy.loginfo("")

    def publish_vel_cmd(self):
        """
        Publishes command velocities computed by the control algorithm
        @param: self
        @result: publish message
        """     
        self.twist = Twist()
        self.twist.linear.x = self.vel_cmd[0]
        self.twist.angular.z = self.vel_cmd[1]
        self.publisher_cmd_vel.publish(self.twist)

    def publish_waypoints(self):
        """
        Publishes the list of waypoints
        @param: self
        @result: publish message
        """     
        self.marker_array = MarkerArray()
        id = 0
        for waypoint in self.waypoints:
            marker = Marker()
            marker.header.frame_id = "map"
            marker.type = marker.SPHERE
            marker.action = marker.ADD
            marker.scale.x = 0.1
            marker.scale.y = 0.1
            marker.scale.z = 0.1
            marker.color.a = 1.0
            marker.color.r = 1.0
            marker.color.g = 0.0
            marker.color.b = 0.0
            marker.pose.orientation.w = 1.0
            marker.pose.position.x = waypoint[0]
            marker.pose.position.y = waypoint[1]
            marker.pose.position.z = 0.05
            marker.id = id
            id += 1
            self.marker_array.markers.append(marker)
        #rospy.loginfo("Publishing wp")
        self.publisher_waypoints.publish(self.marker_array)

    def onOdom(self, odom_msg):
        """
        Callback function, handling incoming odometry messages.
        @param: self
        @param odom_msg - odometry geometry message
        @result: update of relevant vehicle state variables
        """
        # Store odometry data    
        self.odometry = odom_msg
        self.position.x = odom_msg.pose.pose.position.x
        self.position.y = odom_msg.pose.pose.position.y
        explicit_quat = [odom_msg.pose.pose.orientation.x, odom_msg.pose.pose.orientation.y, odom_msg.pose.pose.orientation.z, odom_msg.pose.pose.orientation.w]
        euler = euler_from_quaternion(explicit_quat)
        self.heading = euler[2]

        # Check if current target has been reached; set next one if necessary and possible        
        if self.isWaypointReached():
            if not self.setNextWaypoint():
                if not self.done:
                    rospy.loginfo("This was the last waypoint in the list.")
                    self.done = True
 
        # Apply PD algorithm       
        self.controller()

        # Publish velocity commands
        self.publish_vel_cmd()

        # Publish waypoint list
        self.publish_waypoints()


if __name__ == '__main__':
    rospy.init_node("Planner")
    controller = PDController()
    controller.run()
