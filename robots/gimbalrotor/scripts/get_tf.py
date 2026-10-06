#!/usr/bin/env python3
#!/usr/bin/env python3
import rospy
import tf
from geometry_msgs.msg import Pose
from nav_msgs.msg import Odometry

class TFPublisher():
    def __init__(self):
        self.listener = tf.TransformListener()
        self.publisher = rospy.Publisher("/gimbalrotor/endeffector_pose", Pose, queue_size=1)
        self.subscriber = rospy.Subscriber("/gimbalrotor/uav/cog/odom", Odometry, self.callback)

    def callback(self, msg):
        (trans, rot) = self.listener.lookupTransform('world', 'gimbalrotor/end_effector', rospy.Time(0))
        rospy.loginfo("Translation: %s", trans)
        rospy.loginfo("Rotation (quaternion): %s", rot)
        print("----------------------------------")
        msg_pub = Pose()
        msg_pub.position.x = trans[0]
        msg_pub.position.y = trans[1]
        msg_pub.position.z = trans[2]
        msg_pub.orientation.x = rot[0]
        msg_pub.orientation.y = rot[1]
        msg_pub.orientation.z = rot[2]
        msg_pub.orientation.w = rot[3]
        self.publisher.publish(msg_pub)

if __name__ == '__main__':
    rospy.init_node('get_tf_node')
    rospy.sleep(10.0)
    publisher = TFPublisher()
    rospy.spin()
