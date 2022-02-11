#! /usr/bin/env python3
'''
automatic landing using acoustic sensors
'''
import copy
import math
import rospy
import actionlib
import numpy as np
from franka_msgs.msg import FrankaState
from std_msgs.msg import Float64MultiArray
from geometry_msgs.msg import PoseStamped
from geometry_msgs.msg import TwistStamped
from rv_msgs.msg import MoveToPoseAction, MoveToPoseGoal
# my lib
# from GetRealSenseData import GetRealSenseData


def rotationMatrixToEulerAngles(R):
  sy = math.sqrt(R[0, 0] * R[0, 0] + R[1, 0] * R[1, 0])
  singular = sy < 1e-6
  if not singular:
    x = math.atan2(R[2, 1], R[2, 2])
    y = math.atan2(-R[2, 0], sy)
    z = math.atan2(R[1, 0], R[0, 0])
  else:
    x = math.atan2(-R[1, 2], R[1, 1])
    y = math.atan2(-R[2, 0], sy)
    z = 0
  # limit angle range
  x = x + 2*math.pi if x < 0 else x
  x = x - 2*math.pi if x >= 2*math.pi else x
  # y = y + 2*math.pi if y < 0 else y
  # y = y - 2*math.pi if y >= 2*math.pi else y
  return np.array([x, y, z])


class FrankaMotion():
  T_O_ee = None
  franka_state_msg = Float64MultiArray()
  vel_msg = TwistStamped()
  vel_msg_old = TwistStamped()
  sensors = []
  normal_vec = []
  max_range = 250.0

  def __init__(self, pub_rate=800):
    rospy.init_node('scan_w_acoustic_ee', anonymous=True)
    rospy.Subscriber('VL53L0X/normal', Float64MultiArray, self.VL53L0X_norm_cb)
    rospy.Subscriber('VL53L0X/distance', Float64MultiArray, self.VL53L0X_dist_cb)
    rospy.Subscriber('franka_state_controller/franka_states', FrankaState, self.franka_ee_cb)
    self.vel_pub = rospy.Publisher('arm/cartesian/velocity', TwistStamped, queue_size=1)
    self.rate = rospy.Rate(pub_rate)
    print('waiting for sensors & robot status ...')
    while not rospy.is_shutdown():
      if len(self.sensors) and self.T_O_ee is not None:
        print('sensors & robot status received')
        break
      self.rate.sleep()

  def home(self):
    client = actionlib.SimpleActionClient('/arm/cartesian/pose', MoveToPoseAction)
    client.wait_for_server()
    target = PoseStamped()
    target.header.frame_id = 'panda_link0'
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

  def land(self):
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
        print('landed on surface')
        break
      self.vel_pub.publish(self.vel_msg)
      self.rate.sleep()

  def keep_vertical_using_normal(self):
    kp = [0.5, 0.5, 0.2]
    dead_zone = [0.0, 0.0, 0.0]
    while not rospy.is_shutdown():
      Vx = self.T_O_ee[:3, 0]
      Vz = self.normal_vec
      Vy = np.cross(Vz, Vx)
      rpy_current = rotationMatrixToEulerAngles(self.T_O_ee[:3, :3])
      rpy_desired = rotationMatrixToEulerAngles(np.array([Vx, Vy, Vz]))
      rpy_desired[-1] = -1.57
      # print(rpy_desired)
      # print(rpy_current)
      rot_err = rpy_desired - rpy_current
      # proportional + low-pass; x,y,z are with respect to eef frame!
      w = 3/8
      if abs(rot_err[0]) >= dead_zone[0]:
        vel_ang_x = w*kp[0]*(rot_err[1]) + (1-w)*self.vel_msg_old.twist.angular.x
      else:
        vel_ang_x = 0
      if abs(rot_err[1]) >= dead_zone[1]:
        vel_ang_y = w*kp[1]*(-rot_err[0]) + (1-w)*self.vel_msg_old.twist.angular.y
      else:
        vel_ang_y = 0
      if abs(rot_err[2]) >= dead_zone[2]:
        vel_ang_z = w*kp[2]*(rot_err[2]) + (1-w)*self.vel_msg_old.twist.angular.z
      else:
        vel_ang_z = 0
      print('rotx_err:{:.2f}\troty_err:{:.2f}\trotx_vel:{:.2f}\troty_vel:{:.2f}'.
            format(rot_err[0], rot_err[1], vel_ang_x, vel_ang_y))
      self.vel_msg.twist.angular.x = vel_ang_x
      self.vel_msg.twist.angular.y = vel_ang_y
      self.vel_msg.twist.angular.z = vel_ang_z
      self.vel_pub.publish(self.vel_msg)
      self.vel_msg_old.twist.angular.x = self.vel_msg.twist.angular.x
      self.vel_msg_old.twist.angular.y = self.vel_msg.twist.angular.y
      self.vel_msg_old.twist.angular.z = self.vel_msg.twist.angular.z
      self.rate.sleep()

  def keep_vertical_using_diff(self):
    # 1-3 in-plane
    # 0-2 out-of-plane
    kp = 0.001
    dead_zone = 3
    while not rospy.is_shutdown():
      in_plane_err = self.sensors[1] - self.sensors[3]
      out_of_plane_err = self.sensors[2] - self.sensors[0]
      print('in-plane err:{:.2f}\tout-of-plane err:{:.2f}'.format(in_plane_err, out_of_plane_err))
      self.vel_msg.twist.angular.x = kp*in_plane_err if abs(in_plane_err) > dead_zone else 0
      self.vel_msg.twist.angular.y = kp*out_of_plane_err if abs(out_of_plane_err) > dead_zone else 0
      self.vel_pub.publish(self.vel_msg)
      self.rate.sleep()

  def VL53L0X_dist_cb(self, msg):
    self.sensors = []
    for i in range(4):
      if ~np.isnan(msg.data[i]):
        self.sensors.append(msg.data[i])
      else:
        self.sensors.append(self.max_range)

  def VL53L0X_norm_cb(self, msg):
    # if ~np.isnan(msg.data[0]) and ~np.isnan(msg.data[1]) and ~np.isnan(msg.data[2]):
    self.normal_vec = []
    self.normal_vec.append(msg.data[0])
    self.normal_vec.append(msg.data[1])
    self.normal_vec.append(msg.data[2])

  def franka_ee_cb(self, msg):
    EE_pos = msg.O_T_EE  # inv 4x4 matrix
    self.T_O_ee = np.array([EE_pos[0:4], EE_pos[4:8], EE_pos[8:12], EE_pos[12:16]]).transpose()


if __name__ == "__main__":
  motion = FrankaMotion()
  # motion.home()
  # motion.land()
  # motion.keep_vertical_using_diff()
  motion.keep_vertical_using_normal()
