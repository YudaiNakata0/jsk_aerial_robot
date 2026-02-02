#!/usr/bin/env python3
import rospy
from geometry_msgs.msg import WrenchStamped
from std_msgs.msg import Bool

class ContactDetector():
    def __init__(self):
        self.setup_parameters()
        self.setup_ros()

    def setup_parameters(self):
        self.xyz_external_wrench = [0]*3
        self.xyz_wrench_command = [0]*3
        self.xyz_external_wrench_ref = [0]*3
        self.if_detecting = False
        self.count = 0
        self.coef = 0.5
        
    def setup_ros(self):
        rospy.init_node("contact_detector_node")
        self.sub_wrench = rospy.Subscriber("/gimbalrotor/filtered_est_external_wrench", WrenchStamped, self.wrench_callback)
        self.sub_wrench_command = rospy.Subscriber("/gimbalrotor/desire_wrench", WrenchStamped, self.wrench_command_callback)
        self.pub_contact_state = rospy.Publisher("/gimbalrotor/contact_state", Bool, queue_size=1)

    def wrench_callback(self, msg):
        self.xyz_external_wrench = [msg.wrench.force.x,
                                    msg.wrench.force.y,
                                    msg.wrench.force.z]

    def wrench_command_callback(self, msg):
        self.xyz_wrench_command = [msg.wrench.force.x,
                                   msg.wrench.force.y,
                                   msg.wrench.force.z]
        # if desire wrench is 0, don't use in contact detection
        if self.xyz_wrench_command[0] == 0:
            self.xyz_wrench_command[0] = -10
        if self.xyz_wrench_command[1] == 0:
            self.xyz_wrench_command[1] = -10
        if self.xyz_wrench_command[2] == 0:
            self.xyz_wrench_command[2] = -10
        self.xyz_external_wrench_ref = self.xyz_external_wrench
        self.if_detecting = True
        self.main()

    def main(self):
        while self.if_detecting:
            self.count = 0.0
            # net wrench (robot generate)
            wrench_diff = [-self.xyz_external_wrench[0] + self.xyz_external_wrench_ref[0],
                           -self.xyz_external_wrench[1] + self.xyz_external_wrench_ref[1],
                           -self.xyz_external_wrench[2] + self.xyz_external_wrench_ref[2],]
            print(wrench_diff)
            if wrench_diff[0] > self.xyz_wrench_command[0] * self.coef and wrench_diff[1] > self.xyz_wrench_command[1] * self.coef and wrench_diff[2] > self.xyz_wrench_command[2] * self.coef:
                self.count += 1
            else:
                self.count = 0

            # publish state
            msg = Bool()
            if self.count > 1000:
                print("contact.")
                msg.data = True
                self.pub_contact_state.publish(msg)
            else:
                msg.data = False
                self.pub_contact_state.publish(msg)

if __name__ == "__main__":
    detector = ContactDetector()
    rospy.spin()
