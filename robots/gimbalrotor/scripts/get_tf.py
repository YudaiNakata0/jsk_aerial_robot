#!/usr/bin/env python3
#!/usr/bin/env python3
import rospy
import tf
from geometry_msgs.msg import Pose

if __name__ == '__main__':
    rospy.init_node('get_tf_node')

    listener = tf.TransformListener()
    publisher = rospy.Publisher("/gimbalrotor/endeffector_pose", Pose, queue_size=1)

    rate = rospy.Rate(1.0)
    while not rospy.is_shutdown():
        try:
            (trans, rot) = listener.lookupTransform('world', 'gimbalrotor/end_effector', rospy.Time(0))
            rospy.loginfo("Translation: %s", trans)
            rospy.loginfo("Rotation (quaternion): %s", rot)
            print("----------------------------------")
            msg = Pose()
            msg.position.x = trans[0]
            msg.position.y = trans[1]
            msg.position.z = trans[2]
            msg.orientation.x = rot[0]
            msg.orientation.y = rot[1]
            msg.orientation.z = rot[2]
            msg.orientation.w = rot[3]
            publisher.publish(msg)
            
        except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
            continue
        rate.sleep()
