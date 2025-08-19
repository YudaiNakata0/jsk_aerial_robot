#!/usr/bin/env python3
#!/usr/bin/env python3
import rospy
import tf

if __name__ == '__main__':
    rospy.init_node('get_tf_node')

    listener = tf.TransformListener()

    rate = rospy.Rate(1.0)
    while not rospy.is_shutdown():
        try:
            (trans, rot) = listener.lookupTransform('world', 'gimbalrotor/end_effector', rospy.Time(0))
            rospy.loginfo("Translation: %s", trans)
            rospy.loginfo("Rotation (quaternion): %s", rot)
            print("----------------------------------")
        except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
            continue
        rate.sleep()
