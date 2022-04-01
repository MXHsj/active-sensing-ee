#! /usr/bin/env python3
'''
record robot state T_O_ee, Wrench
'''
import os
import csv
import rospy
import numpy as np
from franka_msgs.msg import FrankaState
from geometry_msgs.msg import Twist
from geometry_msgs.msg import WrenchStamped


def msg2matrix(raw_msg):
  # convert 12x1 message array to SE(3) matrix
  if np.isnan(raw_msg[0]):
    T = None
  else:
    T = np.array([[raw_msg[0], raw_msg[3], raw_msg[6], raw_msg[9]],
                  [raw_msg[1], raw_msg[4], raw_msg[7], raw_msg[10]],
                  [raw_msg[2], raw_msg[5], raw_msg[8], raw_msg[11]],
                  [0.0, 0.0, 0.0, 1.0]])
  return T


def franka_pose_cb(msg):
  EE_pos = msg.O_T_EE_d  # inv 4x4 matrix
  global T_O_ee
  T_O_ee = np.array([EE_pos[0:4], EE_pos[4:8], EE_pos[8:12], EE_pos[12:16]]).transpose()


def franka_force_cb(msg):
  global Fx, Fy, Fz
  Fx = msg.wrench.force.x
  Fy = msg.wrench.force.y
  Fz = msg.wrench.force.z


def joystick_axes_cb(msg):
  # TODO: record teleop command
  pass


if __name__ == "__main__":
  freq = 30         # loop rate
  T_O_ee = None
  Fx = None
  Fy = None
  Fz = None
  rospy.init_node('franka_state_logger', anonymous=True)
  rospy.Subscriber('franka_state_controller/franka_states', FrankaState, franka_pose_cb)
  rospy.Subscriber('/franka_state_controller/F_ext', WrenchStamped, franka_force_cb)
  rospy.Subscriber('cmd_js', Twist, joystick_axes_cb)
  file_path = os.path.join(os.path.dirname(__file__), '../data/franka_state.csv')
  file_out = open(file_path, 'w')
  writer = csv.writer(file_out)
  rate = rospy.Rate(freq)
  print('connecting to robot ...')
  while not rospy.is_shutdown():
    if T_O_ee is not None and Fx is not None:
      break
  while not rospy.is_shutdown():
    data = T_O_ee.flatten()
    data = np.append(data, Fx)
    data = np.append(data, Fy)
    data = np.append(data, Fz)
    print('eef wrench x: {:.3f} y: {:.3f} z: {:.3f} \n'.format(Fx, Fy, Fz))
    writer.writerow(data)
    rate.sleep()
  print('robot pose written to file ', file_path)
  file_out.close()
