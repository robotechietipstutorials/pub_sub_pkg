#!/usr/bin/env python3

import rospy
from pub_sub_pkg.srv import MultiplyTwoInts
from std_msgs.msg import Int32

def handle_add_two_ints(request):
    response = request.x * request.y
    rospy.loginfo(f"Multiplication: {request.x} * {request.y} = {response}")
    return response

def mul_service():
    rospy.init_node('multiplication_service')
    s = rospy.Service('multiply_two_ints', MultiplyTwoInts, handle_add_two_ints)
    rospy.loginfo("Ready to multiphy two ints.")
    rospy.spin()

if __name__ == "__main__":
    mul_service()