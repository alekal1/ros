#!/usr/bin/env python2
import rospy
from sensor_msgs.msg import Imu
from std_msgs.msg import Int16
from geometry_msgs.msg import Twist
import tf
 
class SteeringNode():
    def __init__(self):
        self.imu_subscriber = rospy.Subscriber('/imu', Imu, self.callback)
        self.distance_subscriber = rospy.Subscriber('/distance', Int16, self.distance_check)
        self.velocity_publisher = rospy.Publisher('/diff_drive_controller/cmd_vel', Twist, queue_size=10)
        self.twist = Twist()
 
    def callback(self, imu):
        orientation = imu.orientation
        orientation_list = [orientation.x, orientation.y, orientation.z, orientation.w]
        (roll, pitch, yaw) = tf.transformations.euler_from_quaternion(orientation_list)
        # imu.linear_acceleration.x = pitch
        # imu.angular_velocity.z = roll
        self.twist.linear.x = pitch
        self.twist.angular.z = -roll
        self.velocity_publisher.publish(self.twist)       
    
    def distance_check(self, distance):
        pass
 
 
    def run(self):
        while not rospy.is_shutdown():
            rospy.spin()
    
if __name__=="__main__":
    rospy.init_node("steering_node")
    steering_node = SteeringNode()
    steering_node.run()