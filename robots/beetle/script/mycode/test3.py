#!/usr/bin/env python3

import rospy
from aerial_robot_msgs.msg import FlightNav

class NodeClass():
    def __init__(self):
        self.sub_r = rospy.Subscriber("/mocap", , callback_robot_position)
        
        self.pub = rospy.Publisher("/beetle1/uav/nav", FlightNav, queue_size=10)

    def callback_robot_position(msg):
        


if __name__=="__main__":
    
