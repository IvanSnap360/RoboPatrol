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
        (0.0, 0.0, 0.0),
        (0.0, 0.0, 0.0,1.0),
        rospy.Time.now(),
        "base_link",
        "point")


while not rospy.is_shutdown():
    main()
    rate.sleep()
