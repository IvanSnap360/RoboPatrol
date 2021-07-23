#! /usr/bin/env python 
import rospy
from geometry_msgs.msg import Twist, Pose, Quaternion, TransformStamped
from sensor_msgs.msg import JointState
from tf import TransformBroadcaster, transformations
import tf
from nav_msgs.msg import Odometry
from numpy import cos, sin

rospy.init_node("odom_driver")

rospy.loginfo("Start odom_driver")

vels = Twist()
pos = Pose()
odom_broadcaster = TransformBroadcaster()

odom = Odometry()

current_time = rospy.Time()
last_time = rospy.Time()

rate = rospy.Rate(10)

joint_freedack_msg = JointState()
joint_freedack_msg.header.stamp = 0.0
joint_freedack_msg.name = ["M1", "M2", "M3", "M4"]
joint_freedack_msg.velocity = [0.0, 0.0, 0.0, 0.0]
joint_freedack_msg.position = [0.0, 0.0, 0.0, 0.0]
joint_freedack_msg.effort = [0.0, 0.0, 0.0, 0.0]



odom_pub = rospy.Publisher("/odom", Odometry, queue_size=50)

def main():
    global last_time
    global current_time
    L = 0.250
    current_time = rospy.Time.now()

    v = vels.linear.x
    w = vels.angular.z

    dt = (current_time - last_time).to_sec()

    dd = v * dt
    dth = w * dt

    dx = dd * cos(pos.orientation.z + (dth /2))
    dy = dd * sin(pos.orientation.z + (dth /2))

    pos.position.x += dx
    pos.position.y += dy
    pos.orientation.z += dth

    odom_quat = transformations.quaternion_from_euler(0, 0, pos.orientation.z)

    odom_trans = TransformStamped()
    odom_trans.header.stamp = current_time
    odom_trans.header.frame_id = "odom"
    odom_trans.child_frame_id = "base_link"

    odom_trans.transform.translation.x = pos.position.x
    odom_trans.transform.translation.y = pos.position.y
    odom_trans.transform.translation.z = 0.0
    odom_trans.transform.rotation.x = odom_quat[0]
    odom_trans.transform.rotation.y = odom_quat[1]
    odom_trans.transform.rotation.z = odom_quat[2]
    odom_trans.transform.rotation.w = odom_quat[3]

    odom_broadcaster.sendTransformMessage(odom_trans)

    odom.header.stamp = current_time
    odom.header.frame_id = "odom"

    odom.pose.pose.position.x = -pos.position.x
    odom.pose.pose.position.y = pos.position.y
    odom.pose.pose.position.z = 0.0
    odom.pose.pose.orientation.x = odom_quat[0]
    odom.pose.pose.orientation.y = odom_quat[1]
    odom.pose.pose.orientation.z = odom_quat[2]
    odom.pose.pose.orientation.w = odom_quat[3]

    odom.child_frame_id = "base_link"
    odom.twist.twist.linear.x = vels.linear.x
    odom.twist.twist.linear.y = vels.linear.y
    odom.twist.twist.angular.z = vels.angular.z

    odom_pub.publish(odom)

    last_time = current_time


last_time = rospy.Time.now()

def joint_state_sub_cb_f(msg):
    global joint_freedack_msg
    joint_freedack_msg = msg

def vels_sub_cb_f(msg):
    global vels
    vels = msg
    main()

joint_sub = rospy.Subscriber("/joint_state", JointState, joint_state_sub_cb_f)
vels_sub = rospy.Subscriber("/velocities", Twist,vels_sub_cb_f)

rospy.spin()
rospy.loginfo("Stop odom_driver")
