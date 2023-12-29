#!/usr/bin/env python3

import rospy
from std_msgs.msg import Int32

def publisher_node():
    rospy.init_node('publisher_node', anonymous=True)
    pub = rospy.Publisher('integer_data', Int32, queue_size=10)
    rate = rospy.Rate(1)
    while not rospy.is_shutdown():
        integer_data = 42  # Replace this with your desired integer value
        pub.publish(integer_data)
        rospy.loginfo(f'Sending: {integer_data}')
        rate.sleep()

if __name__ == '__main__':
    try:
        publisher_node()
    except rospy.ROSInterruptException:
        pass