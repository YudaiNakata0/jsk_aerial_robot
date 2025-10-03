#!/usr/bin/env python3
import rospy
from enum import Enum, auto
from geometry_msgs.msg import Pose, PoseStamped
from std_msgs.msg import Int8
from nav_msgs.msg import Odometry
from BridgingStateBase import BridgingStateBase, WaitState, MoveState, AttachState, ExtrudeState
from module import operation_quaternion as oq

class BridgingClass():
    class State(Enum):
        WAITSTATE = 1
        MOVESTATE = auto()
        ATTACHSTATE = auto()
        EXTRUDESTATE = auto()
        
    
    def __init__(self):
        self.CurrentState = WaitState(self)
        self.initialize_parameters()
        self.setup_ros()

    def change_state(self, NextState):
        self.CurrentState.on_exit()
        self.CurrentState = NextState
        self.CurrentState.on_enter()

    def handle_event(self, event, data=None):
        self.CurrentState.handle_event(event, data)

    def initialize_parameters(self):
        self.cog_pose = Pose()
        self.cog_yaw = 0.0
        self.cog_goal_pose = Pose()
        self.endeffector_pose = Pose()

    def setup_ros(self):
        self.subscriber_cog_pose = rospy.Subscriber("/gimbalrotor/uav/cog/odom", Odometry, self.cb_get_cog_pose)
        self.tfBuffer = tf2_ros.Buffer()
        self.listener = tf2_ros.TransformListener(self.tfBuffer)
        self.timer_endeffector_pose = rospy.Timer(rospy.Duration(0.1), self.cb_get_endeffector_pose)
        self.publisher_goal = rospy.Publisher("/gimbalrotor/target_pose", PoseStamped, queue_size=1)
        self.publisher_endeffector_pose = rospy.Publisher("/gimbalrotor/endeffector_pose", Pose, queue_size=1)
        self.publisher_state = rospy.Publisher("/bridging_state", Int8, queue_size=1)
        self.timer_state = rospy.Timer(rospy.Duration(0.1), self.cb_publish_state)

    def cb_publish_state(self, event):
        state_num = self.State[self.CurrentState.__class__.__name__.upper()].value
        self.publisher_state.publish(state_num)

    def cb_get_cog_pose(self, msg):
        self.cog_pose = msg.pose.pose
        angles = oq.quaternion_to_euler(self.cog_pose.orientation)
        self.cog_yaw = angles[2]

    def cb_get_endeffector_pose(self, event):
        self.get_endeffector_pose()
    def get_endeffector_pose(self):
        try:
            trans = self.tfBuffer.lookup_transform("world", "gimbalrotor/end_effector", rospy.Time(0))
            self.endeffector_pose.position = trans.transform.translation
            self.endeffector_pose.orientation = trans.transform.rotation
            self.publisher_endeffector_pose.publish(self.endeffector_pose)
        except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException):
            return

if __name__ == "__main__":
    rospy.init_node("bridging_node")
    BridgingClass = BridgingClass()
    rospy.spin()
