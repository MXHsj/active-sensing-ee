#! /usr/bin/env python3
'''
test franka motion
'''
import rospy
import actionlib
import numpy as np
from franka_msgs.msg import FrankaState
from std_msgs.msg import Float64MultiArray
from geometry_msgs.msg import TwistStamped
from geometry_msgs.msg import PoseStamped
from geometry_msgs.msg import TwistStamped
from rv_msgs.msg import MoveToPoseAction, MoveToPoseGoal


class Test():
  T_O_ee = None
  vel_msg = TwistStamped()
  vel_msg.twist.linear.x = 0.0
  vel_msg.twist.linear.y = 0.0
  vel_msg.twist.linear.z = 0.0
  vel_msg.twist.angular.x = 0.0
  vel_msg.twist.angular.y = 0.0
  vel_msg.twist.angular.z = 0.0

  def __init__(self, pub_rate=1000):
    rospy.init_node('test_motion', anonymous=True)
    rospy.Subscriber('franka_state_controller/franka_states', FrankaState, self.franka_ee_cb)
    self.vel_pub = rospy.Publisher('arm/cartesian/velocity', TwistStamped, queue_size=1)
    self.rate = rospy.Rate(pub_rate)

  def goHome(self):
    client = actionlib.SimpleActionClient('/arm/cartesian/pose', MoveToPoseAction)
    client.wait_for_server()
    target = PoseStamped()
    target.header.frame_id = 'panda_link0'
    # define entry pose & scan length
    target.pose.position.x = 0.37
    target.pose.position.y = 0.0
    target.pose.position.z = 0.009
    target.pose.orientation.x = 1.0
    target.pose.orientation.y = 0.0
    target.pose.orientation.z = 0.00
    target.pose.orientation.w = 0.00
    goal = MoveToPoseGoal(goal_pose=target)
    # Send goal and wait for it to finish
    client.send_goal(goal)
    client.wait_for_result()

  def move(self):
    while not rospy.is_shutdown():
      self.vel_msg.twist.linear.x = 0.001
      self.vel_pub.publish(self.vel_msg)
      self.rate.sleep()

  def franka_ee_cb(self, msg):
    EE_pos = msg.O_T_EE  # inv 4x4 matrix
    self.T_O_ee = np.array([EE_pos[0:4], EE_pos[4:8], EE_pos[8:12], EE_pos[12:16]]).transpose()


if __name__ == "__main__":
  action = Test()
  action.goHome()
  # action.move()
