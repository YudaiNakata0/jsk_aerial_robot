#!/use/bin/env python3
import rospy
from geometry_msgs.msg import Vector3, Pose, PoseStamped
from module import operation_quaternion as oq
from module import operation_vector3 as ov

class BridgingStateBase():
    def __init__(self, ref, d=0.337, h=0.195, v=0.1):
        self.ref = ref
        self.l_xy = d
        self.l_z = h
        self.mean_velocity = v

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
    def __init__(self, ref, endeffector_goal_pose):
        super().__init__(ref)
        self.direction = Vector3()
        self.direction_yaw = 0.0
        self.distance = 0.0
        self.endeffector_goal_pose = endeffector_goal_pose
    
    def on_enter(self):
        self.calculate_direction()
        msg = self.generate_message()
        self.ref.publisher_goal.publish(msg)

    def on_exit(self):
        pass

    def handle_event(self):
        pass

    def calculate_direction(self):
        self.direction.x = ov.get_difference(self.endeffector_goal_pose.position, self.ref.endeffector_pose.position)
        angles = oq.quaternion_to_euler(self.endeffector_goal_pose.orientation)
        self.direction_yaw = angles[2]
        self.distance = ov.get_norm(self.direction)
    
    def generate_message(self):
        msg = PoseStamped()
        msg.pose = self.calculate_pose_endeffector_to_cog()
        duration = self.distance / self.mean_velocity
        
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


        
