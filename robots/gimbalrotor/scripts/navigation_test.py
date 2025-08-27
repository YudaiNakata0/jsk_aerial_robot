#!/usr/bin/env python3
import rospy
from module import Navigation

if __name__ == "__main__":
    rospy.init_node("move_test_node")
    Class = Navigation.Navigation()
    while not rospy.is_shutdown():
        rospy.sleep(0.1)
