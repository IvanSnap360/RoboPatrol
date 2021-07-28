#! /usr/bin/env python 
import rospy
import tf
from tf import transformations
import math

rospy.init_node("tf_rs_base_link_node")

rate = rospy.Rate(30)

broadcaster = tf.TransformBroadcaster()

def main():
    broadcaster.sendTransform(
        (0.110, 0.0, 0.110),
        (0.0, 0.0, 0.0,1.0),
        rospy.Time.now(),
        "camera_link",
        "base_link")


while not rospy.is_shutdown():
    main()
    rate.sleep()
