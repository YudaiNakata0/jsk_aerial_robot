#!/usr/bin/env python
import sys
import os
sys.path.append(os.path.join(os.path.dirname(__file__), '../..'))
sys.path.append(os.path.join(os.path.dirname(__file__), '..'))
import rospy
import smach
import smach_ros
import time
import math
import threading
import numpy as np
from aerial_robot_msgs.msg import FlightNav
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Odometry
from tf.transformations import euler_from_quaternion
from beetle.assembly_api import AssembleDemo
from beetle.disassembly_api import DisassembleDemo
from task.assembly_motion import *
from trajectory import *
from motion_controller import *

class AssembleState(smach.State):
    def __init__(self):

        smach.State.__init__(self, outcomes=['succeeded', 'failed'])

        # Module IDs
        modules_str = rospy.get_param("~module_ids", "")
        real_machine = rospy.get_param("~real_machine", True)
        modules = []
        if modules_str:
            modules = [int(x) for x in modules_str.split(',')]
        else:
            rospy.logerr("No module ID is designated!")

        # AssembleDemo 
        self.assemble_demo = AssemblyDemo(module_ids=modules, real_machine=real_machine)
    
    def execute(self, userdata):
        rospy.loginfo("Assembling UAVs...")
        try:
            self.assemble_demo.main()
            rospy.loginfo("Assembly completed successfully.")
            return 'succeeded'
        except rospy.ROSInterruptException:
            rospy.logerr("Assembly process failed.")
            return 'failed'


def main():
    rospy.init_node('valve_task_state_machine')
    sm = smach.StateMachine(outcomes=['TASK_COMPLETED', 'TASK_FAILED'])
    with sm:
        smach.StateMachine.add('ASSEMBLE', AssembleState(),
                                 transitions={'succeeded': 'TASK_COMPLETED',
                                              'failed': 'TASK_FAILED'})
    outcome = sm.execute()

if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass

