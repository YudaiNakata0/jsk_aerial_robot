#!/usr/bin/env python3
import rospy
from gazebo_msgs.srv import ApplyBodyWrench
from geometry_msgs.msg import Wrench, Vector3
from rospy import Duration, Time

def main():
    rospy.init_node("apply_wrench_client")

    rospy.wait_for_service('/gazebo/apply_body_wrench')
    try:
        apply_wrench = rospy.ServiceProxy('/gazebo/apply_body_wrench', ApplyBodyWrench)

        #--- サービス呼び出し ---
        resp = apply_wrench(
            body_name="gimbalrotor::root",
            reference_frame="world",
            reference_point=Vector3(0.0, 0.0, 0.0),
            wrench=Wrench(
                force=Vector3(-1.0, -1.0, -10.0),
                torque=Vector3(0.0, 0.0, 0.0)
            ),
            start_time=Time(0, 0),
            duration=Duration(1000.0)
        )

        print("Service response:", resp)
    except rospy.ServiceException as e:
        print("Service call failed:", e)

if __name__ == "__main__":
    main()
