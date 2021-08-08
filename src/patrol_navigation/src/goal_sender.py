#! /usr/bin/env python 
import rospy 
from move_base_msgs.msg import MoveBaseAction,MoveBaseGoal
import actionlib

rospy.init_node("goal_sender")

msg = MoveBaseGoal()

msg.target_pose.header.frame_id = "map"
msg.target_pose.header.stamp = rospy.Time.now()

msg.target_pose.pose.position.x = 0.0
msg.target_pose.pose.position.y = 0.0
msg.target_pose.pose.position.z = 0.0

msg.target_pose.pose.orientation.x = 0.0
msg.target_pose.pose.orientation.y = 0.0
msg.target_pose.pose.orientation.z = 0.0
msg.target_pose.pose.orientation.w = 1.0


client = actionlib.SimpleActionClient("move_base",MoveBaseAction)
client.wait_for_server()

client.send_goal(msg)
client.wait_for_result()

print(client.get_state())
