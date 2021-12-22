#! /usr/bin/env python3
'''
automatic landing using acoustic sensors
'''
import time
import rospy
import actionlib
import numpy as np
from franka_msgs.msg import FrankaState
from std_msgs.msg import Float64MultiArray
from geometry_msgs.msg import TwistStamped
from geometry_msgs.msg import PoseStamped
from geometry_msgs.msg import TwistStamped
from rv_msgs.msg import MoveToPoseAction, MoveToPoseGoal


class Landing():
  T_O_ee = None
  franka_state_msg = Float64MultiArray()
  vel_msg = TwistStamped()
  vel_msg.twist.linear.x = 0.0
  vel_msg.twist.linear.y = 0.0
  vel_msg.twist.linear.z = 0.0
  vel_msg.twist.angular.x = 0.0
  vel_msg.twist.angular.y = 0.0
  vel_msg.twist.angular.z = 0.0
  n_sensors = 4
  sensors = []

  def __init__(self, pub_rate=1000):
    rospy.init_node('acoustic_sensor_automatic_landing', anonymous=True)
    rospy.Subscriber('franka_state_controller/franka_states', FrankaState, self.franka_ee_cb)
    rospy.Subscriber('ch101', Float64MultiArray, self.ch101_cb)
    self.vel_pub = rospy.Publisher('arm/cartesian/velocity', TwistStamped, queue_size=1)
    self.rate = rospy.Rate(pub_rate)
    print('waiting for sensors ...')
    while not rospy.is_shutdown():
      if len(self.sensors):
        print('sensors received')
        break
      self.rate.sleep()

  def goHome(self):
    client = actionlib.SimpleActionClient('/arm/cartesian/pose', MoveToPoseAction)
    client.wait_for_server()
    target = PoseStamped()
    target.header.frame_id = 'panda_link0'
    # define entry pose & scan length
    target.pose.position.x = 0.37
    target.pose.position.y = 0.0
    target.pose.position.z = 0.30
    target.pose.orientation.x = -0.7071
    target.pose.orientation.y = 0.7071
    target.pose.orientation.z = 0.00
    target.pose.orientation.w = 0.00
    goal = MoveToPoseGoal(goal_pose=target)
    # Send goal and wait for it to finish
    client.send_goal(goal)
    client.wait_for_result()

  def doLanding(self):
    pause_count = 0
    while not rospy.is_shutdown():
      if self.T_O_ee is not None:
        height = np.sort(np.array(self.sensors))[0]
        print(self.sensors)
        print(height)
        if height > 134 or np.isnan(height):
          self.vel_msg.twist.linear.z = -0.005
        else:
          self.vel_msg.twist.linear.z = 0.0
      else:
        pass
      pause_count += 1
      if pause_count > 200:
        # time.sleep(1)
        pause_count = 0
      self.vel_pub.publish(self.vel_msg)
      self.rate.sleep()

  def ch101_cb(self, msg):
    self.sensors = []
    for i in range(self.n_sensors):
      self.sensors.append(msg.data[i])

  def franka_ee_cb(self, msg):
    EE_pos = msg.O_T_EE  # inv 4x4 matrix
    self.T_O_ee = np.array([EE_pos[0:4], EE_pos[4:8], EE_pos[8:12], EE_pos[12:16]]).transpose()


if __name__ == "__main__":
  action = Landing()
  action.goHome()
  action.doLanding()
