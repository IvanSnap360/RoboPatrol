#! /usr/bin/env python3 
import rospy
from geometry_msgs.msg import Twist
from sensor_msgs.msg import JointState
import numpy as np
import time

rospy.init_node("joint_control_driver")

rate = rospy.Rate(25)

joint_msg = JointState()
joint_msg.header.stamp = 0.0
joint_msg.name = ["M1", "M2", "M3", "M4"]
joint_msg.velocity = [0.0, 0.0, 0.0, 0.0]
joint_msg.position = [0.0, 0.0, 0.0, 0.0]
joint_msg.effort = [0.0, 0.0, 0.0, 0.0]


__L1__ = 0.41 / 2
__L2__ = 0.42 / 2
__D__ = 0.15
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
def joint_state_sub_cb_f(msg):
    global joint_freedack_msg
    joint_freedack_msg = msg


joint_sub = rospy.Subscriber("/joint_state", JointState,joint_state_sub_cb_f)
vels_pub = rospy.Publisher("/velocities", Twist, queue_size=50)



def cmd_vel_f(msg):
    global linear
    global angular

    linear =  msg.linear.x
    angular = msg.angular.z


rospy.Subscriber("/cmd_vel", Twist, cmd_vel_f)


def main():
    global linear
    global angular

    # if (angular < 0 and angular > -3.0):
    #     angular = -3.0
    # elif (angular > 0 and angular < 3.0):
    #     angular = 3.0

    # if (linear < 0 and linear > -0.4):
    #     linear = -0.4
    # elif (linear > 0 and linear < 0.4):
    #     linear = 0.4

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
    C = C * (1/__r__)# 


    solvetion[0] = C[0][0]
    solvetion[1] = C[1][0]
    solvetion[2] = C[2][0]
    solvetion[3] = C[3][0]

    # return list of angular velocities in view [LF, LB, RF, RB]

    joint_msg.velocity[0] = solvetion[0]
    joint_msg.velocity[1] = solvetion[1]
    joint_msg.velocity[2] = solvetion[2]
    joint_msg.velocity[3] = solvetion[3]

    joint_pub.publish(joint_msg)


def compute_vels():
    w1 = joint_freedack_msg.velocity[0]
    w2 = joint_freedack_msg.velocity[1]
    w3 = joint_freedack_msg.velocity[3]
    w4 = joint_freedack_msg.velocity[2]

    rpm1 = joint_freedack_msg.effort[0]
    rpm2 = joint_freedack_msg.effort[1]
    rpm3 = joint_freedack_msg.effort[2]
    rpm4 = joint_freedack_msg.effort[3]

    vels_msg = Twist()

    # vl = (w1 * __D__) / 2
    # vr = (w2 * __D__) / 2

    # v_linear = (vr + vl) / 2
    # v_angular = (vr - vl) / (__L2__ * 2)
    # wheel_circumference_ = np.pi * __D__

    # v_linear = 0
    # v_angular = 0

    # average_rps_x = ((float)(rpm1 + rpm2 + rpm3 + rpm4) / 4) / 60
    # linear_x = average_rps_x * wheel_circumference_

    # average_rps_a = ((float)(-rpm1 + rpm2 - rpm3 + rpm4) / 4) / 60
    # angular_z =  (average_rps_a * wheel_circumference_) / ((__L1__) + (__L2__))

    linear_x = (w1 + w2 + w3 + w4) * (__r__ / 4)
    angular_z = (-w1 + w2 - w3 + w4) * (__r__ / (4 * (__L1__ +  __L2__)))

    # A = np.array([
    #     [1,1,1,1],
    #     [-1,1,1,-1],
    #     [-1/(__L1__ + __L2__), 1/(__L1__ + __L2__), -1/(__L1__ + __L2__), 1/(__L1__ + __L2__)]
    # ], dtype=np.float)

    # B = np.array([[w1],[w2],[w3],[w4]],dtype=np.float)

    # C = A.dot(B)
    # C *= (__r__ / 4)
    
    # linear_x = C[1][0]
    # angular_z = C[2][0]

    # wl = (w1 + w3) / 2
    # wr = (w2 + w4) / 2

    # vl = (wl * __D__) / 2
    # vr = (wr * __D__) / 2

    # linear_x = (vl + vr) / 2
    # angular_z = (vr - vl) / (__L2__ * 2)


    vels_msg.linear.x = round(linear_x,3)
    vels_msg.angular.z = round(angular_z,3)

    vels_pub.publish(vels_msg)
       

while not rospy.is_shutdown():
    main()
    compute_vels()
    rate.sleep()
