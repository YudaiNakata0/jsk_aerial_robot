import rospy
from geometry_msgs.msg import Pose, PoseStamped
from gazebo_msgs.srv import SpawnModel

class GazeboTrajectory:
    def __init__(self):
        rospy.init_node("gazebo_trajectory")

        rospy.wait_for_service("/gazebo/spawn_sdf_model")
        self.spawn_srv = rospy.ServiceProxy(
            "/gazebo/spawn_sdf_model",
            SpawnModel
        )

        rospy.Subscriber(
            "gimbalrotor/endeffector_pose",
            Pose,
            self.cb
        )

        self.count = 0

    def cb(self, msg):
        model_name = "traj_point_%d" % self.count

        sdf = f"""
<sdf version='1.6'>
  <model name='{model_name}'>
    <static>true</static>
    <link name='link'>
      <visual name='visual'>
        <geometry>
          <sphere><radius>0.01</radius></sphere>
        </geometry>
        <material>
          <ambient>1 0 0 1</ambient>
        </material>
      </visual>
    </link>
  </model>
</sdf>
"""

        self.spawn_srv(
            model_name,
            sdf,
            "",
            msg,
            "world"
        )

        self.count += 1

if __name__ == "__main__":
    try:
        GazeboTrajectory()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
