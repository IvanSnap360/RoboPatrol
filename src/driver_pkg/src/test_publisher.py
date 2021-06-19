from mavros_msgs.msg import ActuatorControl
from pymavlink import mavutil

import rospy
from time import sleep
rospy.init_node("test_pub")
mavlink_pub = rospy.Publisher('/mavros/actuator_control', ActuatorControl, queue_size=10)

ros_msg = ActuatorControl()
ros_msg.group_mix = ActuatorControl.PX4_MIX_MANUAL_PASSTHROUGH
ros_msg.controls[0] = 10000
ros_msg.controls[1] = 10000
ros_msg.controls[2] = 10000
ros_msg.controls[3] = 10000
ros_msg.controls[4] = 10000
ros_msg.controls[5] = 10000
ros_msg.controls[6] = 10000
ros_msg.controls[7] = 10000

while not rospy.is_shutdown():
    mavlink_pub.publish(ros_msg)    
    sleep(0.005)