#!/usr/bin/env python
import rospy
from geometry_msgs.msg import PoseStamped
import math

def test_target_pose_publish():
    rospy.init_node("test_target_pose_node", anonymous=True)

    pub_beetle1 = rospy.Publisher("/beetle1/target_pose", PoseStamped, queue_size=10)
    pub_beetle2 = rospy.Publisher("/beetle2/target_pose", PoseStamped, queue_size=10)

    rate_hz = 50  
    rate = rospy.Rate(rate_hz)
    start_time = rospy.Time.now().to_sec()

    rospy.loginfo("Start to pub msg /beetle1/target_pose and /beetle2/target_pose ...")
    
    while not rospy.is_shutdown():
        current_time = rospy.Time.now().to_sec()
        t = current_time - start_time

        msg1 = PoseStamped()
        msg1.header.stamp = rospy.Time.now()
        msg1.header.frame_id = "world"
        msg1.pose.position.x = -1.0 + 0.5 * math.sin(2 * math.pi * 0.5 * t)
        msg1.pose.position.y = -1.0
        msg1.pose.position.z = 1.0

        msg2 = PoseStamped()
        msg2.header.stamp = rospy.Time.now()
        msg2.header.frame_id = "world"
        msg2.pose.position.x = -0.5 + 0.5 * math.cos(2 * math.pi * 0.5 * t)
        msg2.pose.position.y = 1.0
        msg2.pose.position.z = 1.0

        pub_beetle1.publish(msg1)
        pub_beetle2.publish(msg2)

        rate.sleep()

if __name__ == '__main__':
    try:
        test_target_pose_publish()
    except rospy.ROSInterruptException:
        pass
