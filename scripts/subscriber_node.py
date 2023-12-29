#!/usr/bin/env python3

import rospy
from std_msgs.msg import Int32

def integer_callback(msg):
    x = msg.data
    x_square = x*x
    rospy.loginfo(f'Got: {x_square}')

def integer_subscriber():
    rospy.init_node('subscriber_node', anonymous=True)
    rospy.Subscriber('integer_data', Int32, integer_callback)
    rospy.spin()

if __name__ == '__main__':
    integer_subscriber()