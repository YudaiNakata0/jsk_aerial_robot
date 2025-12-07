#!/usr/bin/env python3
import rospy
from geometry_msgs.msg import PoseStamped
import sys

def main():
    rospy.init_node('target_pose_publisher')
    pub = rospy.Publisher('/gimbalrotor/target_pose', PoseStamped, queue_size=10)

    rospy.loginfo("Enter xyz as: x y z")

    while not rospy.is_shutdown():
        line = sys.stdin.readline()
        if not line:
            continue

        try:
            x, y, z = map(float, line.split())
        except ValueError:
            rospy.logwarn("Invalid input. Enter x y z")
            continue

        msg = PoseStamped()
        msg.header.frame_id = "world"
        msg.header.stamp = rospy.Time.now() + rospy.Duration(10.0)

        msg.pose.position.x = x
        msg.pose.position.y = y
        msg.pose.position.z = z

        msg.pose.orientation.w = 1.0

        if x == 0 and y == 0 and z == 0:
            return
        elif z < 0.8:
            rospy.logwarn("too low z value")
        else:
            pub.publish(msg)
            rospy.loginfo("Published PoseStamped: (%.2f, %.2f, %.2f) time=now+10s",
                          x, y, z)

if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass
    
