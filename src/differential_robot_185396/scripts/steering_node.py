#!/usr/bin/env python
import rospy
from sensor_msgs.msg import Imu
from std_msgs.msg import Int16
from geometry_msgs.msg import Twist
 
import tf
 
class Steering():
    def __init__(self):
        self.steering_node_subscriber = rospy.Subscriber('/imu', Imu, self.callback)
        self.steering_node_subscriber_distance = rospy.Subscriber('/distance', Int16, self.distance_check)
        self.velocity_publisher = rospy.Publisher('/this_diff_drive_controller/cmd_vel', Twist, queue_size=10)
        self.twist = Twist()
 
    def callback(self, imu_message):
        orientation = imu_message.orientation
        orientation_list = [
            orientation.x,
            orientation.y,
            orientation.z,
            orientation.w
            ]
        roll, pitch, yaw = tf.transformations.euler_from_quaternion(orientation_list)
        imu_message.linear_acceleration.x = pitch
        imu_message.angular_velocity.z = roll
        publish(pitch, roll)

    def publish(self, pitch, roll):
        self.twist.linear.x = pitch
        self.twist.angular.z = -roll
        self.velocity_publisher.publish(self.twist)       
 
 
    def run(self):
        while not rospy.is_shutdown():
            rospy.spin()

    def distance_check(self, distance):
        pass
    
if __name__=="__main__":
    rospy.init_node("steering_node")
    steering_node = Steering()
    steering_node.run()
 