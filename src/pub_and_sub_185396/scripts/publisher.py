#!/usr/bin/env python
import rospy
from std_msgs.msg import String

def publisher():
    pub = rospy.Publisher('/student_code', String, queue_size=10)
    rospy.init_node('publisher', anonymous=True)
    rate = rospy.Rate(1) # Once every second
    while not rospy.is_shutdown():
        student_code = "185396IAIB_%s" % rospy.get_time()
        rospy.loginfo(student_code)
        pub.publish(student_code)
        rate.sleep()

if __name__ == '__main__':
    try:
        publisher()
    except rospy.ROSInterruptException:
        pass