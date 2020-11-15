#!/usr/bin/env python2

"""
Templete for home assignment 7 (Robot Control).
Node to take a set of waypoints and to drive a differential
drive robot through those waypoints using a simple PD controller
and provided odometry data.

Students should complete the code. Note the places marked with "# TODO".

@author: 
@date: 
@input: Odometry as nav_msgs Odometry message
@output: body velocity commands as geometry_msgs Twist message;
         list of waypoints as MarkerArray message
"""

import math
import rospy
import numpy as np
from geometry_msgs.msg import Twist, Point
from visualization_msgs.msg import MarkerArray, Marker
from nav_msgs.msg import Odometry
from tf.transformations import euler_from_quaternion

angle_threshold = 0.01 #  [rad] if error in heading is greater than this value, the robot will turn on the spot


class PDController:
    def __init__(self):
        # Registering start time of this node
        self.startTime = rospy.Time.now().to_sec()
        # Get the static parameters from the parameter server
        # (which have been loaded onto the server from the yaml file using the launch file).
        # The namespace defined in the launch file must match the namespace used here (i.e., "controller_waypoints")
    
        self.Kd = rospy.get_param("/controller_waypoints/controller/Kd")
        self.Kp = rospy.get_param("/controller_waypoints/controller/Kp")

        self.distance_margin = rospy.get_param("/controller_waypoints/mission/distance_margin")
        self.waypoints = rospy.get_param("/controller_waypoints/mission/waypoints")

        # Print the parameters
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
        # Initialization of class variables
        self.wpIndex = 0          # counts the visited waypoints
        self.position = Point()   # current position of robot;         Point (3D vector: .x, .y, .z)
        self.heading = 0.0        # current orientation of robot (rad yaw)
        self.done = False
        self.init = True
        self.vel_cmd = [10.0, -8.0]  # calculated velocities (linear, angular)
        # Publishers and subscribers
        self.publisher_cmd_vel = rospy.Publisher("/diff_drive_controller/cmd_vel",Twist,queue_size=10)
        self.publisher_waypoints = rospy.Publisher("/mission_control/waypoints",MarkerArray,queue_size=10)
        rospy.Subscriber('odom', Odometry, self.onOdom)
        # Messages
        self.marker_array = None
        self.twist = None
        self.linear_errors = []
        self.angular_errors = []
        self.times = []
        self.command_linear = None
        self.command_angular = None
        self.rsopySl = False


    def wrapAngle(self, angle):
        """
        Helper function that returns angle wrapped between +- Pi.
        Hint: Pass your error in heading [rad] into this function, and it returns the
        shorter angle. This prevents your robot from turning along the wider angle and
        makes it turn along the smaller angle (but in opposite direction) instead.
        @param: self
        @param: angle - angle to be wrapped in [rad]
        @result: returns wrapped angle -Pi <= angle <= Pi
        """        
        if angle > math.pi or angle < -1*math.pi:
            return -(2*math.pi - angle)

    def rad2deg(self, angle):
        """
        Helper function to transform an angle from radians to degrees.
        @param: self
        @param: angle - angle in [rad]
        @result: returns angle in [deg]
        """
        angle = 180.0 * angle / math.pi
        return angle

    def run(self):
        """
        Main loop of class.
        Since we are using a callback function (onOdom) to trigger our computations and outputs,
        we don't need a main loop here. But then we must ensure the node does not terminate.
        @param: self
        @result: runs the step function for controller update
        """        
        # spin() simply keeps python from exiting until this node is stopped
        rospy.loginfo("Waiting for odom message...")
        rospy.spin()

        

    def setNextWaypoint(self):
        """
        Removes current waypoint from list and sets next one as the current target.
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
        Checks if waypoint is reached based on pre-defined threshold.
        @param: self
        @result: returns True if waypoint is reached, otherwise False
        """
        if not self.waypoints:
            return False
        # TODO: Calculate "distance"
        self.diff_x = self.waypoints[0][0] - self.position[0] 
        self.diff_y = self.waypoints[0][1] - self.position[1]
        self.linear_error = self.diff_x + self.diff_y
        self.angular_error = math.atan2(self.diff_y, self.diff_x) - self.heading
        self.linear_errors.append(self.linear_error)
        self.angular_errors.append(self.angular_error)
        self.times.append(rospy.Time.now().to_sec())

        if len(self.linear_errors) > 2:
            self.linear_errors.pop(0)
            self.angular_errors.pop(0)
            self.times.pop(0)

        if len(self.linear_errors) == 2:
            linear_dif = self.linear_errors[1] - self.linear_errors[0]
            angular_dif = self.angular_errors[1] - self.angular_errors[0]
            time_dif = abs(self.times[1] - self.times[0])

            if time_dif == 0:
                self.command_linear = (self.Kp[0] * self.linear_error) + (self.Kd[0] * linear_dif)
                self.command_angular = (self.Kp[0] * self.angular_error) + (self.Kd[0] * angular_dif)
            else:
                self.command_linear = (self.Kp[0] * self.linear_error) + (self.Kd[0] * linear_dif / time_dif)
                self.command_angular = (self.Kp[0] * self.angular_error) + (self.Kd[0] * angular_dif / time_dif)
            if self.command_angular > 1:
                self.command_linear = 0.0

        if abs(self.rad2deg(self.angular_error)) > 2:
             self.command_linear = 0.0
             self.command_angular = self.command_angular * 2

        if self.linear_error < self.distance_margin:
            print("YES")
            self.vel_cmd = [0.0, 0.0]
            self.publish_vel_cmd()
            return True        
        return False

    def controller(self):
        """
        Takes the errors and calculates velocities from it, according to control algorithm specs.
        @param: self
        @result: sets the values in self.vel_cmd
        """
        # Output 0 (skip all calculations) if the last waypoint was reached
        if self.done:
            self.vel_cmd = [0.0,0.0]
            return
        elif self.command_linear is not None:
            self.vel_cmd = [self.command_linear * 2, self.command_angular]

    def publish_vel_cmd(self):
        """
        Publishes command velocities computed by the control algorithm.
        @param: self
        @result: publish message
        """     
        self.twist = Twist()
        # TODO: Your code here
        self.twist.linear.x = self.vel_cmd[0]
        self.twist.angular.z = self.vel_cmd[1] 
        self.publisher_cmd_vel.publish(self.twist)

    def publish_waypoints(self):
        """
        Publishes the list of waypoints, so RViz can see them.
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
        self.publisher_waypoints.publish(self.marker_array)

    def onOdom(self, odom_msg):
        """
        Callback function, handling incoming odometry messages.
        @param: self
        @param odom_msg - odometry geometry message
        @result: update of relevant vehicle state variables
        """
        # Store odometry data    
        # TODO: Store position from odom message in "self.position"    
        self.position = [odom_msg.pose.pose.position.x, odom_msg.pose.pose.position.y, odom_msg.pose.pose.position.z]
        explicit_quat = [odom_msg.pose.pose.orientation.x, odom_msg.pose.pose.orientation.y, odom_msg.pose.pose.orientation.z, odom_msg.pose.pose.orientation.w]
        roll, pich, yaw = euler_from_quaternion(explicit_quat)
        # TODO: Store yaw (heading) from "euler" in "self.heading"
        self.heading = yaw
        # Check if current target has been reached; set next one if necessary and possible  
        if self.isWaypointReached():
            if not self.setNextWaypoint():
                if not self.done:
                    rospy.loginfo("This was the last waypoint in the list.")
                    endTime = rospy.Time.now().to_sec()
                    rospy.loginfo("Started node  [s]: %.2f",self.startTime)
                    rospy.loginfo("Finished node [s]: %.2f",endTime)
                    totalTime = endTime - self.startTime
                    rospy.loginfo("Elapsed time  [s]: %.2f",totalTime)
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
