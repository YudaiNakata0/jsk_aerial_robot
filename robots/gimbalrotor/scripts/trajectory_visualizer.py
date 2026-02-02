#!/usr/bin/env python3
import rospy
import math
from geometry_msgs.msg import Pose
from std_msgs.msg import Bool
from gazebo_msgs.srv import SpawnModel
from tf.transformations import quaternion_about_axis

class PoseTrailCylinder:
    def __init__(self):
        rospy.init_node("trajectory_spwner")

        self.prev_pose = None
        self.cylinder_id = 0
        self.isDrawing = False
        self.duration = 1.0
        self.prev_time = 0.0

        rospy.loginfo("Waiting for /gazebo/spawn_sdf_model ...")
        rospy.wait_for_service("/gazebo/spawn_sdf_model")
        self.spawn_srv = rospy.ServiceProxy(
            "/gazebo/spawn_sdf_model",
            SpawnModel
        )

        self.sub = rospy.Subscriber(
            "/gimbalrotor/endeffector_pose",
            Pose,
            self.pose_callback,
            queue_size=10
        )
        
        self.sub_state = rospy.Subscriber(
            "/drawing_state",
            Bool,
            self.state_callback,
            queue_size=10
        )

        rospy.loginfo("PoseTrailCylinder node started")

    def state_callback(self, msg):
        self.isDrawing = msg.data

    def pose_callback(self, msg):
        if not self.isDrawing:
            self.prev_pose = None
            return

        time = rospy.get_time()
        if time - self.prev_time < self.duration:
            return
        else:
            self.prev_time = time
            
        if self.prev_pose is None:
            self.prev_pose = msg
            return

        p0 = self.prev_pose.position
        p1 = msg.position

        dx = p1.x - p0.x
        dy = p1.y - p0.y
        dz = p1.z - p0.z

        length = math.sqrt(dx*dx + dy*dy + dz*dz)
        if length < 1e-4:
            return

        # 中点
        mid = [(p0.x + p1.x) * 0.5,
               (p0.y + p1.y) * 0.5,
               (p0.z + p1.z) * 0.5]

        # 姿勢計算
        q = self.calc_orientation(dx, dy, dz, length)

        pose = Pose()
        pose.position.x = mid[0]
        pose.position.y = mid[1]
        pose.position.z = mid[2]
        pose.orientation.x = q[0]
        pose.orientation.y = q[1]
        pose.orientation.z = q[2]
        pose.orientation.w = q[3]

        self.spawn_cylinder(pose, length)

        self.prev_pose = msg

    def calc_orientation(self, dx, dy, dz, length):
        """Z軸を (dx,dy,dz) に合わせるクォータニオン"""
        dir_vec = [dx/length, dy/length, dz/length]
        z_axis = [0.0, 0.0, 1.0]

        axis = [
            z_axis[1]*dir_vec[2] - z_axis[2]*dir_vec[1],
            z_axis[2]*dir_vec[0] - z_axis[0]*dir_vec[2],
            z_axis[0]*dir_vec[1] - z_axis[1]*dir_vec[0]
        ]

        axis_norm = math.sqrt(axis[0]**2 + axis[1]**2 + axis[2]**2)

        if axis_norm < 1e-6:
            return [0, 0, 0, 1]

        axis = [a/axis_norm for a in axis]

        dot = (z_axis[0]*dir_vec[0] +
               z_axis[1]*dir_vec[1] +
               z_axis[2]*dir_vec[2])

        dot = max(min(dot, 1.0), -1.0)
        angle = math.acos(dot)

        return quaternion_about_axis(angle, axis)

    def spawn_cylinder(self, pose, length):
        sdf = f"""
<sdf version='1.6'>
  <model name='cylinder_{self.cylinder_id}'>
    <static>true</static>
    <link name='link'>
      <visual name='visual'>
        <geometry>
          <cylinder>
            <radius>0.001</radius>
            <length>{length}</length>
          </cylinder>
        </geometry>
        <material>
            <ambient>1.0 1.0 0.0 1</ambient>
            <diffuse>1.0 1.0 0.0 1</diffuse>
            <specular>0.5 0.5 0.0 1</specular>
        </material>
      </visual>
    </link>
  </model>
</sdf>
"""

        try:
            self.spawn_srv(
                model_name=f"cylinder_{self.cylinder_id}",
                model_xml=sdf,
                robot_namespace="",
                initial_pose=pose,
                reference_frame="world"
            )
            self.cylinder_id += 1
        except rospy.ServiceException as e:
            rospy.logerr(f"Spawn failed: {e}")

if __name__ == "__main__":
    PoseTrailCylinder()
    rospy.spin()
