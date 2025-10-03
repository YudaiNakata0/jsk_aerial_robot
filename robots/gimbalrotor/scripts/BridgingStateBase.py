#!/use/bin/env python3
import rospy
from geometry_msgs.msg import Pose, PoseStamped

class BridgingStateBase():
    def __init__(self, ref):
        self.ref = ref

    def on_enter(self): pass
    def on_exit(self): pass
    def handle_event(self, event): pass
    def on_update(self): pass

class WaitState(BridgingStateBase):
    def __init__(self, ref):
        super().__init__(ref)
        self.subscriber_goal = rospy.Subscriber("/set_goal_pose", Pose, self.cb_goal)

    def on_exit(self):
        self.subscriber_goal.unregister()
        
    def handle_event(self, msg):
        self.ref.goal_pose = msg
        self.ref.change_state(MoveState(self.ref, msg))

    def cb_goal(self, msg):
        self.handle_event(msg)

class MoveState(BridgingStateBase):
    def __init__(self, ref, goal_pose):
        super().__init__(ref)
        self.goal_pose = goal_pose
    
    def on_enter(self):
        msg = PoseStamped()
        msg.pose = self.goal_pose
        self.ref.publisher_goal.publish(msg)
        pass

    def on_exit(self):
        pass

    def handle_event(self):
        pass

        
class AttachState(BridgingStateBase):
    def on_enter(self):
        pass

    def on_exit(self):
        pass

    def handle_event(self):
        pass


class ExtrudeState(BridgingStateBase):
    def on_enter(self):
        pass

    def on_exit(self):
        pass

    def handle_event(self):
        pass


        
