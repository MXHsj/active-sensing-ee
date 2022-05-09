#! /usr/bin/env python3
'''
record robot state T_O_ee, Wrench
'''
import os
import csv
import rospy
import numpy as np
from cv2 import cv2
from datetime import date
from cv_bridge import CvBridge
from sensor_msgs.msg import Image
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


def clarius_us_cb(msg):
  global US_Bmode
  US_Bmode = CvBridge().imgmsg_to_cv2(msg, desired_encoding="passthrough")


def franka_pose_cb(msg):
  global T_O_ee
  EE_pos = msg.O_T_EE_d  # inv 4x4 matrix
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
  freq = 5         # loop rate
  isRecUS = True    # flag to enable recording US images
  US_Bmode = None
  T_O_ee = None
  Fx = None
  Fy = None
  Fz = None
  rospy.init_node('franka_state_logger', anonymous=True)
  rospy.Subscriber('franka_state_controller/franka_states', FrankaState, franka_pose_cb)
  rospy.Subscriber('/franka_state_controller/F_ext', WrenchStamped, franka_force_cb)
  rospy.Subscriber('cmd_js', Twist, joystick_axes_cb)
  rospy.Subscriber('/Clarius/US', Image, clarius_us_cb)
  robot_logger_path = os.path.join(os.path.dirname(__file__), '../data/franka_state/franka_state.csv')
  image_logger_path = os.path.join(os.path.dirname(__file__), '../data/clarius/lung-{}/'.format(date.today()))
  if not os.path.exists(image_logger_path):
    os.makedirs(image_logger_path)
  robot_logger = open(robot_logger_path, 'w')
  writer = csv.writer(robot_logger)
  rate = rospy.Rate(freq)

  # wait for robot state to be received
  print('connecting to robot ...')
  while not rospy.is_shutdown():
    if isRecUS:
      if T_O_ee is not None and Fx is not None and US_Bmode is not None:
        break
    else:
      if T_O_ee is not None and Fx is not None:
        break

  # record data
  counter = 0
  while not rospy.is_shutdown():
    data = T_O_ee.flatten()
    data = np.append(data, Fx)
    data = np.append(data, Fy)
    data = np.append(data, Fz)
    print('i: {} ee force x: {:.3f} y: {:.3f} z: {:.3f} \n'.format(counter, Fx, Fy, Fz))
    writer.writerow(data)
    if isRecUS:
      US_Bmode_gray = cv2.cvtColor(US_Bmode, cv2.COLOR_BGR2GRAY)
      cv2.imwrite(os.path.join(image_logger_path, 'US_{}.jpg'.format(counter)), US_Bmode_gray)
    counter += 1
    rate.sleep()

  print('robot pose written to file ', robot_logger_path)
  if isRecUS:
    print('US images written to ', image_logger_path)
  robot_logger.close()
