#!/usr/bin/env python3
import rospy
from enum import Enum, auto
from geometry_msgs.msg import Pose, PoseStamped
from std_msgs.msg import Int8
from BridgingStateBase import BridgingStateBase, WaitState, MoveState, AttachState, ExtrudeState

class BridgingClass():
    class State(Enum):
        WAITSTATE = 1
        MOVESTATE = auto()
        ATTACHSTATE = auto()
        EXTRUDESTATE = auto()
        
    
    def __init__(self):
        self.CurrentState = WaitState(self)
        self.ros_setup()

    def change_state(self, NextState):
        self.CurrentState.on_exit()
        self.CurrentState = NextState
        self.CurrentState.on_enter()

    def handle_event(self, event, data=None):
        self.CurrentState.handle_event(event, data)

    def ros_setup(self):
        self.publisher_goal = rospy.Publisher("/gimbalrotor/target_pose", PoseStamped, queue_size=1)
        self.publisher_state = rospy.Publisher("/bridging_state", Int8, queue_size=1)
        self.timer_state = rospy.Timer(rospy.Duration(0.1), self.cb_publish_state)

    def cb_publish_state(self, event):
        state_name = self.State[self.CurrentState.__class__.__name__.upper()].value
        self.publisher_state.publish(state_name)
        

if __name__ == "__main__":
    rospy.init_node("bridging_node")
    BridgingClass = BridgingClass()
    rospy.spin()
