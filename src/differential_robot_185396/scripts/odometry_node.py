#!/usr/bin/env python
import rospy
import math
from nav_msgs.msg import Odometry
from std_msgs.msg import Header
from gazebo_msgs.srv import GetModelState, GetModelStateRequest
from differential_robot_185396.msg import counter_message

list_of_times = []
dict_of_encoders = {}
right_encoders_values = []
left_encoders_values = []
diff_betweeen_messages = 0

cpr = 1440 # counts per second
radians_left = 0
radians_right = 0

left_wheel_speed = 0
right_wheel_speed = 0

odom = Odometry()
header = Header()
header.frame_id = '/odom'
header.child_frame = 'base_link'

def callback(msgs):
    """
    Callback method.
    """

    # Initialize global variables
    global dict_of_encoders
    global right_encoders_values
    global left_encoders_values
    global diff_betweeen_messages

    calculate_time_between_messages() # Calculate time between messages
    calculate_wheels_rotation_velocities(msgs.count_left, msgs.count_right)
    calculate_wheels_linear_velocities()
    calculate_robot_linear_velocity()
    calculate_robot_angular_velocity()


def calculate_time_between_messages():
    """
    Calculates time between messages.

    """

     # Make global variables
    global list_of_times
    global diff_betweeen_messages

    now = rospy.get_time() # Get time
    list_of_times.append(now) # Store the time in list

    diff_betweeen_messages = tick_calculation(list_of_times)
    # rospy.loginfo(rospy.get_caller_id() + " Difference between messages: %s", diff)
    if len(list_of_times) > 2:
        list_of_times = list_of_times[-1:] # Clears list


def calculate_wheels_rotation_velocities(left_encoder, right_encoder):
    global cpr
    global radians_left
    global radians_right
    global left_wheel_speed
    global right_wheel_speed

    """
    Method for rotation velocities calculation.
    """
    right_encoders_values.append(right_encoder) # Store right encoders values in list
    left_encoders_values.append(left_encoder) # Store left encoders values in list
    
    # Store all encoders in dict
    dict_of_encoders['left'] = left_encoders_values
    dict_of_encoders['right'] = right_encoders_values

    # Get right values
    right_values = dict_of_encoders['right']

    # Get left values
    left_values = dict_of_encoders['left']

    # Calculate right ticks
    r_tick = tick_calculation(right_values)
    if r_tick > 1000:
        r_tick = cpr - r_tick

    # Calculate left ticks
    l_tick = tick_calculation(left_values)
    if l_tick > 1000:
        l_tick = cpr - l_tick

    # Because time betwen messages are small, we need to handle when it's value is 0
    try:
        radians_left = calculate_radians(l_tick)
        radians_right = calculate_radians(r_tick)
        left_wheel_speed = radians_left / diff_betweeen_messages
        right_wheel_speed  = radians_right / diff_betweeen_messages

        dict_of_encoders['right'] = right_values[-1:]
        dict_of_encoders['left'] = left_values[-1:]
    except ZeroDivisionError:
        pass

    # rospy.loginfo("Left speed %s", left_wheel_speed)
    # rospy.loginfo("Right speed %s", right_wheel_speed)

def calculate_wheels_linear_velocities():
    """
    Method to calculate linear velocities of both wheels
    """
    global cpr
    global radians_left
    global radians_right

    wheel_radius = 0.075

    linear_velocity_left = radians_left * wheel_radius
    linear_velocity_right = radians_right * wheel_radius

    # rospy.loginfo("Linear velocity left %s m/s", linear_velocity_left)
    # rospy.loginfo("Linear velocity right %s m/s", linear_velocity_right)
    return linear_velocity_left, linear_velocity_right

def calculate_robot_linear_velocity():
    """
    Method to calculate robot linear velocity.
    """

    global left_wheel_speed
    global right_wheel_speed

    # rospy.loginfo("Robot linear velocity %s", (right_wheel_speed - left_wheel_speed) / 2)
    return (right_wheel_speed - left_wheel_speed) / 2

def calculate_robot_angular_velocity():
    """
    Method to calculate robot angular velocity.
    """

    distance_between_wheels = 0.165 # Distance between wheels in m
    # rospy.loginfo("Robot angular velocity %s", (right_wheel_speed - left_wheel_speed) / distance_between_wheels)
    return (right_wheel_speed - left_wheel_speed) / distance_between_wheels


def calculate_radians(val):
    """
    Method for radian calculation.
    """

    return ((val/4) * math.pi) / 100


def tick_calculation(values):
    """
    Method for calculation.
    """

    try:
        return abs(values[-1] - values[-2])
    except IndexError:
        return 0


def run():
    global odom
    global header
    """
    Main method.
    """

    rospy.Subscriber('/encoders_output', counter_message, callback)

    rospy.init_node('odometry_node', anonymous=True)
    odom_pub = rospy.Publisher('/odom', Odometry)
    r = rospy.Rate(2)
    while not rospy.is_shutdown():
        

    rospy.spin()


# rospy.init_node('odom_pub', anonymous=True)
# odom_pub = rospy.Publisher("/odom", Odometry)

# rospy.wait_for_service('/gazebo/get_model_state')
# get_model_srv = rospy.ServiceProxy('/gazebo/get_model_state', GetModelState)

# odom = Odometry()
# header = Header()
# header.frame_id = '/odom'

# model = GetModelStateRequest()
# model.model_name = "robot"

# r = rospy.Rate(2)

# while not rospy.is_shutdown():

#     result = get_model_srv(model)
#     odom.pose.pose = result.pose
#     odom.twist.twist = result.twist

#     header.stamp = rospy.Time.now()
#     odom.header = header

#     odom_pub.publish(odom)

#     r.sleep()

if __name__ == '__main__':
    run()