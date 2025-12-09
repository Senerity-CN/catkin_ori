#!/usr/bin/env python

import rospy
import tf
from nav_msgs.msg import Odometry

class OdomTFPublisher:
    def __init__(self):
        rospy.init_node('odom_tf_publisher')
        self.tf_broadcaster = tf.TransformBroadcaster()
        self.odom_sub = rospy.Subscriber('/ugv/odometry', Odometry, self.odom_callback)
        rospy.loginfo("Odom TF Publisher started")

    def odom_callback(self, msg):
        # Extract position and orientation from odometry message
        x = msg.pose.pose.position.x
        y = msg.pose.pose.position.y
        z = msg.pose.pose.position.z
        
        qx = msg.pose.pose.orientation.x
        qy = msg.pose.pose.orientation.y
        qz = msg.pose.pose.orientation.z
        qw = msg.pose.pose.orientation.w
        
        # Publish TF transform from odom to base_link
        self.tf_broadcaster.sendTransform(
            (x, y, z),
            (qx, qy, qz, qw),
            rospy.Time.now(),
            "base_link",
            "odom"
        )

if __name__ == '__main__':
    try:
        publisher = OdomTFPublisher()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass