#!/usr/bin/env python3
import rospy
import sys
import termios
import tty
import select
import time
from geometry_msgs.msg import WrenchStamped
from std_msgs.msg import Bool, Int8

def get_key():
    if select.select([sys.stdin], [], [], 0)[0]:
        return sys.stdin.read(1)
    return None

def main():
    if not sys.stdin.isatty():
        print("!run in terminal!")
        return

    old_settings = termios.tcgetattr(sys.stdin)
    tty.setcbreak(sys.stdin.fileno())
    
    rospy.init_node("set_desire_wrench_node")

    rate = rospy.Rate(50)

    mode = 0 # 0: world frame, 1: body frame
    
    try:
        while not rospy.is_shutdown():
            key = get_key()

            if key is None:
                rate.sleep()
                continue
            


    finally:
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, old_settings)
        print("terminal restored. bye.")

if __name__ == "__main__":
    main()
