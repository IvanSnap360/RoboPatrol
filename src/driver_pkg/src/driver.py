#! /usr/bin/env python3
import rospy
from geometry_msgs.msg import Twist
from sensor_msgs.msg import JointState
import numpy as np
import time

rospy.init_node("joint_control_driver")

rate = rospy.Rate(10)

joint_msg = JointState()
joint_msg.header.stamp = 0.0
joint_msg.name = ["M1", "M2", "M3", "M4"]
joint_msg.velocity = [0.0, 0.0, 0.0, 0.0]
joint_msg.position = [0.0, 0.0, 0.0, 0.0]
joint_msg.effort = [0.0, 0.0, 0.0, 0.0]


__L1__ = 0.175 / 2
__L2__ = 0.245 / 2
__D__ = 0.13 
__r__ = __D__ / 2

linear = 0
angular = 0

joint_pub = rospy.Publisher("/joint_contol", JointState, queue_size=50)

joint_freedack_msg = JointState()
joint_freedack_msg.header.stamp = 0.0
joint_freedack_msg.name = ["M1", "M2", "M3", "M4"]
joint_freedack_msg.velocity = [0.0, 0.0, 0.0, 0.0]
joint_freedack_msg.position = [0.0, 0.0, 0.0, 0.0]
joint_freedack_msg.effort = [0.0, 0.0, 0.0, 0.0]
def joint_state_sub_cb_f(msg: JointState):
    global joint_freedack_msg
    joint_freedack_msg = msg


joint_sub = rospy.Subscriber("/joint_state", JointState,joint_state_sub_cb_f)
vels_pub = rospy.Publisher("/velocities", Twist, queue_size=50)


def cmd_vel_f(msg: Twist):
    global linear
    global angular

    linear = msg.linear.x
    angular = msg.angular.z


rospy.Subscriber("/cmd_vel", Twist, cmd_vel_f)


def main():
    global linear
    global angular

    joint_msg.header.stamp = rospy.Time.now()

    solvetion = [0.0, 0.0, 0.0, 0.0]

    A = np.array([
        [1., -1., -(__L1__ + __L2__)],
        [1.,  1.,  (__L1__ + __L2__)],
        [1.,  1., -(__L1__ + __L2__)],
        [1., -1.,  (__L1__ + __L2__)]
    ], dtype=np.float)

    B = np.array([[linear], [0.0], [angular]], dtype=np.float)

    C = A.dot(B)
    C = C * (1/__r__)

    solvetion[0] = C[0][0]
    solvetion[1] = C[1][0]
    solvetion[2] = C[3][0]
    solvetion[3] = C[2][0]

    # return list of angular velocities in view [LF, LB, RF, RB]

    joint_msg.velocity[0] = solvetion[0]
    joint_msg.velocity[1] = solvetion[1]
    joint_msg.velocity[2] = solvetion[2]
    joint_msg.velocity[3] = solvetion[3]

    joint_pub.publish(joint_msg)


def compute_vels():
    w1 = joint_freedack_msg.velocity[0]
    w2 = joint_freedack_msg.velocity[1]
    w3 = joint_freedack_msg.velocity[2]
    w4 = joint_freedack_msg.velocity[3]

    vels_msg = Twist()

    vl = (((w1)) * __D__) / 2 
    vr = (((w2)) * __D__) / 2 

    vels_msg.linear.x = (vr + vl) / 2
    vels_msg.angular.z = (vr - vl) / (__L2__ * 2)

    # vels_msg.linear.x = (w1 + w2 + w3 + w4) * __r__/4
    # vels_msg.linear.y = (-w1 + w2 + w3 - w4) * r/4
    # vels_msg.angular.z = (-w1 + w2 - w3 + w4) * (__r__ / 4 * (__L1__ + __L2__))

    vels_pub.publish(vels_msg)


while not rospy.is_shutdown():
    main()
    compute_vels()
    rate.sleep()
