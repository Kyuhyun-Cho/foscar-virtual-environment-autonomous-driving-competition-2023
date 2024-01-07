#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import rospy
from nav_msgs.msg import Odometry
from geometry_msgs.msg import TransformStamped
import tf2_ros

class PubTF:
    def __init__(self):
        rospy.init_node('odom_tf_broadcast', anonymous=True)
        self.br = tf2_ros.TransformBroadcaster()
        self.t = TransformStamped()
        rospy.Subscriber('odom', Odometry, self.callback)

    def callback(self, msg):
        self.t.header.frame_id = 'odom'
        self.t.header.stamp = rospy.Time.now()
        self.t.child_frame_id = msg.child_frame_id
        self.t.transform.translation.x = msg.pose.pose.position.x
        self.t.transform.translation.y = msg.pose.pose.position.y
        self.t.transform.translation.z = msg.pose.pose.position.z
        self.t.transform.rotation = msg.pose.pose.orientation
        self.br.sendTransform(self.t)

def main():
    try:
        pub_tf=PubTF()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass

if __name__ == '__main__':
    main()