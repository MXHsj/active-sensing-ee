#! /usr/bin/env python3
'''
Name        : teleop_joystick
Author      : Xihan Ma
Version     : 
Description : control 3-DOF franka end-effector using flight joystick
'''
import math
import copy
import rospy
import numpy as np
from std_msgs.msg import Bool
from sensor_msgs.msg import Joy
from geometry_msgs.msg import Twist
from geometry_msgs.msg import Wrench
from franka_msgs.msg import FrankaState
from geometry_msgs.msg import WrenchStamped


class Teleop:
  def __init__(self):
    rospy.init_node('franka_teleop_joy')
    # joystick buttons/axes
    self.ax0dzone = rospy.get_param('~ax0deadzone', 0.03)
    self.ax1dzone = rospy.get_param('~ax1deadzone', 0.03)
    self.ax4dzone = rospy.get_param('~ax4deadzone', 0.05)
    self.cmd = None
    self.js = Joy()
    self.js.axes = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
    self.js.buttons = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
                       0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
    # master & slave motion
    self.T_O_ee = np.array([[0.0, -1.0, 0.0, 0.0],
                            [-1.0, 0.0, 0.0, 0.0],
                            [0.0, 0.0, -1.0, 0.0],
                            [0.35, 0.0, 0.35, 1.0]]).transpose()
    self.curr_slave = Twist()
    self.last_slave = Twist()
    self.curr_master = Twist()
    self.last_master = Twist()
    self.d_slave = Twist()
    self.d_master = Twist()
    self.isContact_msg = Bool()
    # tele operation states
    self.isContact = False          # force control along approach vector
    self.isAbort = False            # abort motion
    self.triggerState = False       # tranlation-rotation switch
    self.triggerStateOld = False
    self.wrench_slave = Wrench()
    self.wrench_slave_old = Wrench()
    self.force_des = 5.5
    self.force_max = 8.0  # max contact force [N]

  def doTeleop(self):
    # ROS stuff
    cmd_pub = rospy.Publisher('cmd_js', Twist, queue_size=1)
    # cmd_pub = rospy.Publisher('franka_cmd_acc', Twist, queue_size=1)
    contact_mode_pub = rospy.Publisher('isContact', Bool, queue_size=1)
    rospy.Subscriber("joy", Joy, self.js_callback)
    rospy.Subscriber('franka_state_controller/franka_states', FrankaState, self.ee_callback)
    rospy.Subscriber('/franka_state_controller/F_ext', WrenchStamped, self.force_callback)
    self.freq = rospy.get_param('~hz', 100)
    rate = rospy.Rate(self.freq)

    while not rospy.is_shutdown():
      # update slave state
      self.curr_slave.linear.x = self.T_O_ee[0, 3]
      self.curr_slave.linear.y = self.T_O_ee[1, 3]
      self.curr_slave.linear.z = self.T_O_ee[2, 3]
      eul = self.rot2rpy(self.T_O_ee[:3, :3])
      eul[0] = eul[0] + 6.28 if eul[0] < 0 else eul[0]    # -pi->pi
      self.curr_slave.angular.x = eul[0]
      self.curr_slave.angular.y = eul[1]
      self.curr_slave.angular.z = eul[2]
      # slave velocity
      self.d_slave.linear.x = (self.curr_slave.linear.x-self.last_slave.linear.x)*self.freq
      self.d_slave.linear.y = (self.curr_slave.linear.y-self.last_slave.linear.y)*self.freq
      self.d_slave.linear.z = (self.curr_slave.linear.z-self.last_slave.linear.z)*self.freq
      self.d_slave.angular.x = (self.curr_slave.angular.x-self.last_slave.angular.x)*self.freq
      self.d_slave.angular.y = (self.curr_slave.angular.y-self.last_slave.angular.y)*self.freq
      self.d_slave.angular.z = (self.curr_slave.angular.z-self.last_slave.angular.z)*self.freq
      # master velocity
      self.d_master.linear.x = (self.curr_master.linear.x-self.last_master.linear.x)*self.freq
      self.d_master.linear.y = (self.curr_master.linear.y-self.last_master.linear.y)*self.freq
      self.d_master.linear.z = (self.curr_master.linear.z-self.last_master.linear.z)*self.freq
      self.d_master.angular.x = (self.curr_master.angular.x-self.last_master.angular.x)*self.freq
      self.d_master.angular.y = (self.curr_master.angular.y-self.last_master.angular.y)*self.freq
      self.d_master.angular.z = (self.curr_master.angular.z-self.last_master.angular.z)*self.freq
      # total force on end-effector
      self.force_sum = math.sqrt(self.wrench_slave.force.x**2 +
                                 self.wrench_slave.force.y**2 +
                                 self.wrench_slave.force.z**2)
      # publish command
      self.checkState()
      self.mapjs2master()
      if self.isAbort:
        self.stopMotion()
      else:
        # TODO: merge direct_vel_mapping with PDcontrol
        if not self.isContact:
          self.cmd = self.direct_vel_mapping()
        elif self.isContact:
          self.cmd = self.PDcontrol()
      if self.cmd:
        cmd_pub.publish(self.cmd)
      self.isContact_msg.data = self.isContact
      contact_mode_pub.publish(self.isContact_msg)
      self.last_slave = copy.deepcopy(self.curr_slave)
      self.last_master = copy.deepcopy(self.curr_master)
      rate.sleep()

  def checkState(self):
    # switch to contact mode
    if self.js.buttons[10] and not self.isContact:
      if abs(self.js.axes[0]) < self.ax0dzone and abs(self.js.axes[1]) < self.ax1dzone and abs(self.js.axes[4]) < self.ax4dzone:
        self.isContact = not self.isContact
        print("Contact mode: ", self.isContact)
      else:
        print("Only switch contact mode at neutral position")
    # switch to non-contact mode
    if self.js.buttons[11] and self.isContact:
      if abs(self.js.axes[0]) < self.ax0dzone and abs(self.js.axes[1]) < self.ax1dzone and abs(self.js.axes[4]) < self.ax4dzone:
        self.isContact = not self.isContact
        print("Contact mode: ", self.isContact)
      else:
        print("Only switch contact mode at neutral position")

    # abort if trigger pressed at non-neutral position
    self.triggerState = self.js.buttons[0]
    if self.triggerState is not self.triggerStateOld:
      if abs(self.js.axes[0]) > 0.1 or abs(self.js.axes[1]) > 0.1:
        print("Abort! move to neutral position!")
        self.isAbort = True
    self.triggerStateOld = self.triggerState

    # resume motion if js at neutral and commands are zeroed
    tol = 1e-5
    if self.isAbort:
      if abs(self.js.axes[0]) < self.ax0dzone and abs(self.js.axes[1]) < self.ax1dzone and abs(self.js.axes[4]) < self.ax4dzone \
              and abs(self.cmd.linear.x < tol) and abs(self.cmd.linear.y < tol) and abs(self.cmd.linear.z < tol) \
              and abs(self.cmd.angular.x < tol) and abs(self.cmd.angular.y < tol) and abs(self.cmd.angular.z < tol):
        print("Ready to move!")
        self.isAbort = False

    # check external force on eef
    if self.force_sum > self.force_max:
      print("exceeding max external force")

  def mapjs2master(self):
    '''map joystick input to desired eef pose'''
    sLin = [0.008, 0.008, 0.006]   # [x, y, z] [0.008, 0.008, 0.006]
    sAng = [0.008, 0.008, 0.008]   # [x, y, z] [0.008, 0.008, 0.01]
    stiff = [7e-4, 7e-4, 7e-4]    # [x, y, z] [6e-4, 6e-4, 7e-4]
    dampz = 1.8e-6
    Vz = self.T_O_ee[:3, 2]		# approach vector
    force_error = self.force_des-self.wrench_slave.force.z
    force_error_d = (self.wrench_slave_old.force.z-self.wrench_slave.force.z)*self.freq

    if self.triggerState:
      # map orientation
      if self.isContact:
        self.curr_master.angular.x = self.curr_slave.angular.x + \
            sAng[0]*self.js.axes[0]*(self.force_sum < self.force_max)
        self.curr_master.angular.y = self.curr_slave.angular.y + \
            sAng[1]*self.js.axes[1]*(self.force_sum < self.force_max)
        self.curr_master.angular.z = self.curr_slave.angular.z + \
            sAng[2]*self.js.axes[4]*(self.force_sum < self.force_max)
      else:
        self.curr_master.angular.x = self.curr_slave.angular.x + sAng[0]*self.js.axes[0]
        self.curr_master.angular.y = self.curr_slave.angular.y + sAng[1]*self.js.axes[1]
        self.curr_master.angular.z = self.curr_slave.angular.z + sAng[2]*self.js.axes[4]
      self.curr_master.linear.x = self.curr_slave.linear.x
      self.curr_master.linear.y = self.curr_slave.linear.y
      self.curr_master.linear.z = self.curr_slave.linear.z
    elif not self.triggerState:
      # map position
      if self.isContact:
        self.curr_master.linear.x = self.curr_slave.linear.x + \
            (sLin[0]/5*self.js.axes[1] + stiff[0]*force_error*Vz[0])
        self.curr_master.linear.y = self.curr_slave.linear.y + \
            (-sLin[1]/5*self.js.axes[0] + stiff[1]*force_error*Vz[1])
        self.curr_master.linear.z = self.curr_slave.linear.z + (stiff[2]*force_error)*Vz[2]
        # print(self.curr_master.linear.z)
      else:
        self.curr_master.linear.x = self.curr_slave.linear.x + (sLin[0]*self.js.axes[1])
        self.curr_master.linear.y = self.curr_slave.linear.y - (sLin[1]*self.js.axes[0])
        self.curr_master.linear.z = self.curr_slave.linear.z - (sLin[2]*self.js.axes[4])
      self.curr_master.angular.x = self.curr_slave.angular.x
      self.curr_master.angular.y = self.curr_slave.angular.y
      self.curr_master.angular.z = self.curr_slave.angular.z

  def PDcontrol(self):
    '''velocity command: u = Kp*(Xm-Xs)'''
    # TODO: separate remove residual function
    u = Twist()
    # linear
    u.linear.x = 1.2*(self.curr_master.linear.x - self.curr_slave.linear.x)
    u.linear.y = 1.2*(self.curr_master.linear.y - self.curr_slave.linear.y)
    u.linear.z = 1.5*(self.curr_master.linear.z - self.curr_slave.linear.z)
    # angular
    u.angular.x = 2.8*(self.curr_master.angular.x - self.curr_slave.angular.x)
    u.angular.y = 3.0*(self.curr_master.angular.y - self.curr_slave.angular.y)
    u.angular.z = 4.0*(self.curr_master.angular.z - self.curr_slave.angular.z)
    # get rid of small motion
    u.angular.x = 0.0 if abs(u.angular.x) < 1e-7 else u.angular.x
    u.angular.y = 0.0 if abs(u.angular.y) < 1e-7 else u.angular.y
    u.angular.z = 0.0 if abs(u.angular.z) < 1e-7 else u.angular.z
    u.linear.x = 0.0 if abs(u.linear.x) < 1e-7 else u.linear.x
    u.linear.y = 0.0 if abs(u.linear.y) < 1e-7 else u.linear.y
    u.linear.z = 0.0 if abs(u.linear.z) < 1e-7 else u.linear.z
    return u

  def direct_vel_mapping(self):
    lin_acc_max = 13.0*0.017
    ang_acc_max = 25.0*0.012
    ang_acc_o2 = -ang_acc_max
    ang_acc_o1 = ang_acc_max
    ang_vel_o3 = (1/3)*ang_acc_o2
    ang_vel_o2 = (1/2)*ang_acc_o1
    lin_acc_o2 = -lin_acc_max
    lin_acc_o1 = lin_acc_max
    lin_vel_o3 = (1/3)*lin_acc_o2
    lin_vel_o2 = (1/2)*lin_acc_o1
    u = Twist()
    if not self.triggerState:
      # linear
      u.linear.x = math.copysign(
          lin_vel_o3*abs(self.js.axes[1])**3 + lin_vel_o2*abs(self.js.axes[1])**2, self.js.axes[1])
      u.linear.y = math.copysign(
          lin_vel_o3*abs(self.js.axes[0])**3 + lin_vel_o2*abs(self.js.axes[0])**2, self.js.axes[0])
      u.linear.z = math.copysign(
          lin_vel_o3*abs(self.js.axes[4])**3 + lin_vel_o2*abs(self.js.axes[4])**2, self.js.axes[4])
    elif self.triggerState:
      # angular
      u.angular.x = math.copysign(
          ang_vel_o3*abs(self.js.axes[0])**3 + ang_vel_o2*abs(self.js.axes[0])**2, self.js.axes[0])
      u.angular.y = math.copysign(
          ang_vel_o3*abs(self.js.axes[1])**3 + ang_vel_o2*abs(self.js.axes[1])**2, self.js.axes[1])
      u.angular.z = math.copysign(
          ang_vel_o3*abs(self.js.axes[4])**3 + ang_vel_o2*abs(self.js.axes[4])**2, self.js.axes[4])
    return u

  def stopMotion(self):
    # decrease velocity command to 0
    incre = 2e-4
    tol = 1e-5
    self.cmd.linear.x -= math.copysign(incre, self.cmd.linear.x) if abs(self.cmd.linear.x) > tol else 0.0
    self.cmd.linear.y -= math.copysign(incre, self.cmd.linear.y) if abs(self.cmd.linear.y) > tol else 0.0
    self.cmd.linear.z -= math.copysign(incre, self.cmd.linear.z) if abs(self.cmd.linear.z) > tol else 0.0
    self.cmd.angular.x -= math.copysign(incre, self.cmd.angular.x) if abs(self.cmd.angular.x) > tol else 0.0
    self.cmd.angular.y -= math.copysign(incre, self.cmd.angular.y) if abs(self.cmd.angular.y) > tol else 0.0
    self.cmd.angular.z -= math.copysign(incre, self.cmd.angular.z) if abs(self.cmd.angular.z) > tol else 0.0

  def js_callback(self, js_msg):
    self.js = js_msg

  def ee_callback(self, ee_msg):
    EE_pos = ee_msg.O_T_EE_d  # inv 4x4 matrix
    self.T_O_ee = np.array([EE_pos[0:4], EE_pos[4:8], EE_pos[8:12], EE_pos[12:16]]).transpose()

  def force_callback(self, FT_msg):
    self.wrench_slave_old = self.wrench_slave
    self.wrench_slave.force.x = 0.0 if FT_msg.wrench.force.x < 0 else FT_msg.wrench.force.x
    self.wrench_slave.force.y = 0.0 if FT_msg.wrench.force.y < 0 else FT_msg.wrench.force.y
    self.wrench_slave.force.z = 0.0 if FT_msg.wrench.force.z < 0 else FT_msg.wrench.force.z

  def rot2rpy(self, R):
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
    return np.array([x, y, z])


if __name__ == "__main__":
  teleop_obj = Teleop()
  teleop_obj.doTeleop()
