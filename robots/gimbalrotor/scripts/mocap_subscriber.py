#!/usr/bin/env python3
import rospy
from geometry_msgs.msg import Pose

class MocapSubscriber():
    def __init__(self, n, i):
        self.robot_ns = n
        self.robot_id = i
        self.flag = 0
        self.msg = Pose()
        self.topic_name = "/" + self.robot_ns + "/pose"
        self.sub = rospy.Subscriber(self.topic_name, Pose, self.cb__goal_pose)
        self.pub = rospy.Publisher("/set_goal_pose", Pose, queue_size=10)
        
    def cb(self, msg):
        if flag:
            return
        self.msg = msg

if __name__ == "__main__":
    rospy.init_node("mocap_subscriber_node")
    Target1 = MocapSubscriber("target1", 11)
    while not rospy.is_shutdown():
        rospy.sleep(0.1)
