#!/usr/bin/env python3

import rospy
import actionlib

from rv_msgs.msg import ServoToPoseAction, ServoToPoseGoal
from geometry_msgs.msg import PoseStamped

# initialise ros node
rospy.init_node('servo_to_points_example')

# Create a ros action client to communicate with the driver
client = actionlib.SimpleActionClient('/arm/cartesian/servo_pose', ServoToPoseAction)
client.wait_for_server()

# Create a target pose
target = PoseStamped()
target.header.frame_id = 'panda_link0'

# Populate with target position/orientation (READY POSE)
target.pose.position.x = 0.000
target.pose.position.y = 0.000
target.pose.position.z = 0.500

target.pose.orientation.x = -1.00
target.pose.orientation.y = 0.00
target.pose.orientation.z = 0.00
target.pose.orientation.w = 0.00

# Create goal from target pose
goal = ServoToPoseGoal(stamped_pose=target, scaling=0.2)

# Send goal and wait for it to finish
client.send_goal(goal)
client.wait_for_result()
