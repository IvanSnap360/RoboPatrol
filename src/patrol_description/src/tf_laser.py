#! /usr/bin/env python3
import rospy
import tf
from tf import transformations
import math

rospy.init_node("tf_base_link_node")

rate = rospy.Rate(30)

broadcaster = tf.TransformBroadcaster()

def main():
    broadcaster.sendTransform(
        (0.0, 0.0, 0.165),
        transformations.quaternion_from_euler(0,0,3.14),
        rospy.Time.now(),
        "laser_base",
        "base_link")


while not rospy.is_shutdown():
    main()
    rate.sleep()
