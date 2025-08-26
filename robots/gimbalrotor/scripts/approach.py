#!/usr/bin/env python3
import rospy
from module import Set_Goal

if __name__ == "__main__":
    rospy.init_node("move_test_node")
    Class = Set_Goal.SetGoal()
    while not rospy.is_shutdown():
        print(Class.state)
        if Class.state == 1:
            rospy.sleep(6.0)
            Class.detach()
        rospy.sleep(0.1)
