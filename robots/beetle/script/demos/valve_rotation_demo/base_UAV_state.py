#!/usr/bin/env python
import rospy
import threading
from geometry_msgs.msg import PoseStamped

class BaseUAVState(object):
    def __init__(self):
        module_ids_str = rospy.get_param("~module_ids", "1,2")
        self.module_ids = [int(x) for x in module_ids_str.split(',')]
        if len(self.module_ids) < 2:
            rospy.logerr("BaseUAVState: At least 2 module IDs are required.")
        self.uav_positions = {}
        self.uav_events = {}
        for mid in self.module_ids:
            self.uav_positions[mid] = None
            self.uav_events[mid] = threading.Event()
            topic = "/beetle{}/mocap/pose".format(mid)
            rospy.Subscriber(topic, PoseStamped, self._make_uav_callback(mid), queue_size=1)

    def _make_uav_callback(self, module_id):
        def callback(msg):
            self.uav_positions[module_id] = msg
            self.uav_events[module_id].set()
        return callback

    def wait_for_uav_positions(self, timeout=5):
        success = True
        for mid in self.module_ids:
            if not self.uav_events[mid].wait(timeout):
                rospy.logwarn("BaseUAVState: Timeout waiting for UAV {} position.".format(mid))
                success = False
        return success

    def get_average_position(self):
        if not all(self.uav_positions[mid] is not None for mid in self.module_ids):
            return None
        x = sum(self.uav_positions[mid].pose.position.x for mid in self.module_ids) / len(self.module_ids)
        y = sum(self.uav_positions[mid].pose.position.y for mid in self.module_ids) / len(self.module_ids)
        z = sum(self.uav_positions[mid].pose.position.z for mid in self.module_ids) / len(self.module_ids)
        avg_pose = PoseStamped()
        avg_pose.header.stamp = rospy.Time.now()
        avg_pose.pose.position.x = x
        avg_pose.pose.position.y = y
        avg_pose.pose.position.z = z
        return avg_pose
