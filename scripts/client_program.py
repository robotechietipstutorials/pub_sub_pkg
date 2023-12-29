#!/usr/bin/env python3

import rospy
from pub_sub_pkg.srv import MultiplyTwoInts
from std_msgs.msg import Int32

def arithmetic_client(x, y):
    rospy.wait_for_service('multiply_two_ints')
    try:
        multiply_two_ints = rospy.ServiceProxy('multiply_two_ints', MultiplyTwoInts)
        response = multiply_two_ints(x, y)
        rospy.loginfo(f"Result: {response.product}")
        return response.product
    except rospy.ServiceException as e:
        rospy.logerr(f"Service call failed: {e}")

if __name__ == "__main__":
    rospy.init_node('arithmetic_client')
    x = 58
    y = 12
    arithmetic_client(x, y)