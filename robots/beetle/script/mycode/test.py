#!/usr/bin/env python3

import rospy
import argparse
from aerial_robot_msgs.msg import FlightNav
from spinal.msg import FourAxisCommand

def move(x, y, z):
    pub = rospy.Publisher("/beetle1/uav/nav", FlightNav, queue_size=10)
    msg = FlightNav()
    msg.pos_xy_nav_mode = 2
    msg.pos_z_nav_mode = 2
    msg.target_pos_x = x
    msg.target_pos_y = y
    msg.target_pos_z = z
    rospy.sleep(1.0)
    pub.publish(msg)

def control_thrust():
    pub = rospy.Publisher("/beetle1/four_axes/command", FourAxisCommand, queue_size=10)
    msg = FourAxisCommand()
    msg.angles = [0.0, 0.0, 0.0]
    msg.base_thrust = [15.0, 5.0, 15.0, 5.0]
    rospy.sleep(1.0)
    pub.publish(msg)
    
if __name__=="__main__":
    parser = argparse.ArgumentParser()
    parser.add_argument("-x", "--x_position", default=0.0)
    parser.add_argument("-y", "--y_position", default=0.0)
    parser.add_argument("-z", "--z_position", default=1.0)
    x = float(parser.parse_args().x_position)
    y = float(parser.parse_args().y_position)
    z = float(parser.parse_args().z_position)
    rospy.init_node("test_node")
    #move(x, y, z)
    control_thrust()
