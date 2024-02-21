from nav_msgs.msg import Odometry
import geometry_msgs
import rospy
import tf2_ros


def handle_odom(tf):
    """publishes the transform and odometry message to the /odom topic."""
    br = tf2_ros.StaticTransformBroadcaster()
    odom_trans = geometry_msgs.msg.TransformStamped()
    odom_trans.header.stamp = rospy.Time.now()
    odom_trans.header.frame_id = "odom"
    odom_trans.child_frame_id = "base_link"
    odom_trans.transform.translation.x = tf.transform.translation.x
    odom_trans.transform.translation.y = tf.transform.translation.y
    odom_trans.transform.translation.z = tf.transform.translation.z
    odom_trans.transform.rotation.x = tf.transform.rotation.x
    odom_trans.transform.rotation.y = tf.transform.rotation.y
    odom_trans.transform.rotation.z = tf.transform.rotation.z
    odom_trans.transform.rotation.w = tf.transform.rotation.w
    br.sendTransform(odom_trans)
    odom = Odometry()
    odom.header.stamp = rospy.Time.now()
    odom.header.frame_id = "odom"
    odom.child_frame_id = "base_link"
    odom.pose.pose.position.x = tf.transform.translation.x
    odom.pose.pose.position.y = tf.transform.translation.y
    odom.pose.pose.position.z = tf.transform.translation.z
    odom.pose.pose.orientation.x = tf.transform.rotation.x
    odom.pose.pose.orientation.y = tf.transform.rotation.y
    odom.pose.pose.orientation.z = tf.transform.rotation.z
    odom.pose.pose.orientation.w = tf.transform.rotation.w
    odom.twist.twist.linear.x = 0.1  # Example linear velocity
    odom.twist.twist.angular.z = 0.05  # Example angular velocity
    pub.publish(odom)


if __name__ == '__main__':
    rospy.init_node('tf_to_odom')
    tfBuffer = tf2_ros.Buffer()
    listener = tf2_ros.TransformListener(tfBuffer)
    pub = rospy.Publisher('odom', Odometry, queue_size=10)
    rate = rospy.Rate(10.0)
    while not rospy.is_shutdown():
        try:
            transform = tfBuffer.lookup_transform('map', 'car1_30m/base_link', rospy.Time())
            handle_odom(transform)
        except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException):
            pass
        rate.sleep()
