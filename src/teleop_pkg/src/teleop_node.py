#! /usr/bin/env python 
import rospy
from geometry_msgs.msg import Twist

rospy.init_node("teleop_node")
vels_pub = rospy.Publisher("/cmd_vel",Twist,queue_size=50)
rate = rospy.Rate(30)
import os
import select
import sys

teleop_type = input("choose teleop configuration (numpad/wasdx)")
if teleop_type == "wasdx":
    forward_key = "w"
    backward_key = "x"
    left_key = "a"
    right_key = "d"
    stop_key = "s"
    info_msg = """
----------------------------------------------------------
TELEOP CONFIGURATION:\033[04m\033[01m\033[36mWASDX\033[0m
        w
    a   s   d
        x

w - forward
x - backward
a - left
d - right
s - stop
----------------------------------------------------------
"""
    print(info_msg)
elif teleop_type == "numpad":
    forward_key = "8"
    backward_key = "2"
    left_key = "4"
    right_key = "6"
    stop_key = "5"
    info_msg = """
----------------------------------------------------------
TELEOP CONFIGURATION:\033[04m\033[01m\033[36mNUMPAD\033[0m
    8
4   5   6
    2

8 - forward
2 - backward
4 - left
6 - right
5 - stop
----------------------------------------------------------
"""
    print(info_msg)

stop_teleop_key = '\x03'


if os.name == 'nt':
    import msvcrt
else:
    import tty
    import termios


if os.name != 'nt':
    settings = termios.tcgetattr(sys.stdin)


linear_step = 0.5
linear_max = 1.5
angular_max = 10
angular_step = 0.5
linear = 0.0
angular = 0.0

def constrain(input, low, high):
    if input < low:
      input = low
    elif input > high:
      input = high
    else:
      input = input

    return input

def getKey():
    if os.name == 'nt':
        return msvcrt.getch()

    tty.setraw(sys.stdin.fileno())
    rlist, _, _ = select.select([sys.stdin], [], [], 0.1)
    if rlist:
        key = sys.stdin.read(1)
    else:
        key = None

    termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
    return key


count = 0

vels = Twist()

if __name__ == "__main__":
    while not rospy.is_shutdown():
        if count == 40:
            print(info_msg)
            count = 0
        key = getKey()
        if key != None:
            if (key == forward_key):
                linear += linear_step
                count += 1
                print(linear, angular)
            if(key == backward_key):
                linear -= linear_step
                count += 1
                print(linear, angular)
            if (key == left_key):
                angular += angular_step
                count += 1
                print(linear, angular)
            if(key == right_key):
                angular -= angular_step
                count += 1
                print(linear, angular)
            if (key == stop_teleop_key):
                print("-------------------EXIT-------------------")
                break            
        elif key == None:
            linear = 0
            angular = 0

        vels.linear.x = constrain(linear,-linear_max,linear_max)
        vels.angular.z = constrain(angular,-angular_max,angular_max)

        vels_pub.publish(vels)
        # rate.sleep()