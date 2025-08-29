#!/usr/bin/env python3
import rospy
import argparse
from module import Navigation

if __name__ == "__main__":
    rospy.init_node("move_test_node")
    parser = argparse.ArgumentParser()
    try:
        #launchファイルからの呼び出し
        mode = rospy.get_param("/move_test_node/execution_mode")
    except KeyError:
        #単体での呼び出し
        parser.add_argument("-m", "--mode", default="0")
        args = parser.parse_args()
        mode = args.mode
    Class = Navigation.Navigation(m=mode)
    while not rospy.is_shutdown():
        rospy.sleep(0.1)
