#! /usr/bin/env python3
'''
3 DoF control of the probe using active sensing end-effector
'''
import copy
import math
import rospy
import actionlib
import numpy as np
from franka_msgs.msg import FrankaState
from std_msgs.msg import Float64MultiArray
from geometry_msgs.msg import Twist
from geometry_msgs.msg import PoseStamped
from geometry_msgs.msg import TwistStamped
from geometry_msgs.msg import WrenchStamped
from rv_msgs.msg import MoveToPoseAction, MoveToPoseGoal


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
  ee_wrench = WrenchStamped()
  dist_vec = []
  normal_vec = []
  max_range = 200.0     # maximum sensing range
  desired_yaw = -1.57   # default eef yaw
  pub_rate = 900        # eef velocity publishing rate
  enDepthComp = True    # enable depth compensation

  def __init__(self):
    rospy.init_node('scan_w_active_sensing_ee', anonymous=True)
    rospy.Subscriber('cmd_js', Twist, self.joystick_cb)
    rospy.Subscriber('VL53L0X/normal', Float64MultiArray, self.VL53L0X_norm_cb)
    rospy.Subscriber('VL53L0X/distance', Float64MultiArray, self.VL53L0X_dist_cb)
    rospy.Subscriber('franka_state_controller/franka_states', FrankaState, self.franka_pose_cb)
    rospy.Subscriber('/franka_state_controller/F_ext', WrenchStamped, self.franka_force_cb)
    self.vel_pub = rospy.Publisher('arm/cartesian/velocity', TwistStamped, queue_size=1)
    self.rate = rospy.Rate(self.pub_rate)
    print('waiting for sensor measurements & robot status ...')
    while not rospy.is_shutdown():
      if len(self.dist_vec) and self.T_O_ee is not None:
        print('sensor measurements & robot status received')
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

  def scan_with_active_ee(self):
    while not rospy.is_shutdown():
      self.keep_vertical()      # update angular velocities
      if self.enDepthComp:
        self.keep_contact()     # update linear velocity along z
      self.vel_pub.publish(self.vel_msg)
      print('tdx:{:.2f}  tdy:{:.2f}  tdz:{:.2f}  rdx:{:.2f}  rdy:{:.2f}  rdz:{:.2f}'.
            format(self.vel_msg.twist.linear.x,
                   self.vel_msg.twist.linear.y,
                   self.vel_msg.twist.linear.z,
                   self.vel_msg.twist.angular.x,
                   self.vel_msg.twist.angular.y,
                   self.vel_msg.twist.angular.z))
      self.vel_msg_old.twist.linear.z = self.vel_msg.twist.linear.z
      self.vel_msg_old.twist.angular.x = self.vel_msg.twist.angular.x
      self.vel_msg_old.twist.angular.y = self.vel_msg.twist.angular.y
      self.vel_msg_old.twist.angular.z = self.vel_msg.twist.angular.z
      self.rate.sleep()

  def keep_vertical(self):
    kp = [0.52, 0.52, 0.2]
    dead_zone = [0.0, 0.0, 0.0]
    w = [3/8, 3/8, 1/8]
    Vx = self.T_O_ee[:3, 0]
    Vz = self.normal_vec
    Vy = np.cross(Vz, Vx)
    rpy_current = rotationMatrixToEulerAngles(self.T_O_ee[:3, :3])
    rpy_desired = rotationMatrixToEulerAngles(np.array([Vx, Vy, Vz]))
    rpy_desired[-1] = self.desired_yaw  # fix yaw angle during normal positioning
    rot_err = rpy_desired - rpy_current
    if abs(rot_err[0]) >= dead_zone[0]:
      vel_ang_x = w[0]*kp[0]*(rot_err[1]) + (1-w[0])*self.vel_msg_old.twist.angular.x
    else:
      vel_ang_x = 0
    if abs(rot_err[1]) >= dead_zone[1]:
      vel_ang_y = w[1]*kp[1]*(-rot_err[0]) + (1-w[1])*self.vel_msg_old.twist.angular.y
    else:
      vel_ang_y = 0
    if abs(rot_err[2]) >= dead_zone[2]:
      vel_ang_z = w[2]*kp[2]*(rot_err[2]) + (1-w[2])*self.vel_msg_old.twist.angular.z
    else:
      vel_ang_z = 0
    self.vel_msg.twist.angular.x = vel_ang_x
    self.vel_msg.twist.angular.y = vel_ang_y
    self.vel_msg.twist.angular.z = vel_ang_z

  def keep_contact(self):
    desired_dist = 115  # desired minimum sensor measurement
    desried_force = 5
    w = 3/8
    vel_lin_z_force = -2.0*(desried_force-self.ee_wrench.wrench.force.z)
    # w_force = 1.0       # weight of the force
    # closest = np.min(self.dist_vec)
    # vel_lin_z_dist = 0.01*(desired_dist-closest)
    # vel_lin_z = w_force*vel_lin_z_force + (1-w_force)*vel_lin_z_dist
    vel_lin_z = vel_lin_z_force
    self.vel_msg.twist.linear.z = w*0.01*vel_lin_z+(1-w)*self.vel_msg_old.twist.linear.z

  def joystick_cb(self, msg):
    self.vel_msg.twist.linear.x = msg.linear.x
    self.vel_msg.twist.linear.y = msg.linear.y
    self.desired_yaw += 0.05*msg.linear.z

  def VL53L0X_dist_cb(self, msg):
    self.dist_vec = []
    for i in range(4):
      if ~np.isnan(msg.data[i]):
        self.dist_vec.append(msg.data[i])
      else:
        self.dist_vec.append(self.max_range)

  def VL53L0X_norm_cb(self, msg):
    # if ~np.isnan(msg.data[0]) and ~np.isnan(msg.data[1]) and ~np.isnan(msg.data[2]):
    self.normal_vec = []
    self.normal_vec.append(msg.data[0])
    self.normal_vec.append(msg.data[1])
    self.normal_vec.append(msg.data[2])

  def franka_force_cb(self, msg):
    self.ee_wrench.wrench.force.x = 0.0 if msg.wrench.force.x < 0 else msg.wrench.force.x
    self.ee_wrench.wrench.force.y = 0.0 if msg.wrench.force.y < 0 else msg.wrench.force.y
    self.ee_wrench.wrench.force.z = 0.0 if msg.wrench.force.z < 0 else msg.wrench.force.z

  def franka_pose_cb(self, msg):
    EE_pos = msg.O_T_EE  # inv 4x4 matrix
    self.T_O_ee = np.array([EE_pos[0:4], EE_pos[4:8], EE_pos[8:12], EE_pos[12:16]]).transpose()


if __name__ == "__main__":
  motion = FrankaMotion()
  # motion.home()
  motion.scan_with_active_ee()
