#! /usr/bin/env python3
'''
scan with active-sensing end-effector, with initial landing using realsense
'''
import math
import rospy
import numpy as np
from cv2 import cv2
from franka_motion import FrankaMotion
from GetRealSenseData import GetRealSenseData


def convert2base(T_cam_tar):
  # convert target from camera frame T_cam_tar to base frame T_O_tar
  global T_O_ee, T_ee_cam
  if T_O_ee is not None:
    T_O_cam = np.matmul(T_O_ee, T_ee_cam)
    T = np.matmul(T_O_cam, T_cam_tar)
  else:
    T = float('nan')*np.ones([4, 4])
    print("no robot info")
  return T


def get_pose(P0, Vz):
  global T_O_tar
  if not np.isnan(Vz[0]):
    Vz[0] = -Vz[0]
    Vz[1] = -Vz[1]
    # x component
    xx = 0.0
    xy = -1.0*math.cos(math.pi/8)
    xz = -(Vz[1]*(xy-P0[1])+Vz[0]*(xx-P0[0]))/Vz[2]+P0[2]
    Vx = np.subtract([xx, xy, xz], P0)
    Vx = Vx/np.linalg.norm(Vx)
    # y component
    Vy = np.cross(Vz, Vx)
    Vy = Vy/np.linalg.norm(Vy)
    # homogenuous transformation
    cam_tar = np.array([Vx, Vy, Vz, P0]).flatten()
    T_cam_tar = np.array([[cam_tar[0], cam_tar[3], cam_tar[6], cam_tar[9]],
                          [cam_tar[1], cam_tar[4], cam_tar[7], cam_tar[10]],
                          [cam_tar[2], cam_tar[5], cam_tar[8], cam_tar[11]],
                          [0.0, 0.0, 0.0, 1.0]])
    # entry pose w.r.t base
    dist_coeff = 0.1      # 0.075
    Pz = np.subtract(P0, [dist_coeff*Vzi for Vzi in Vz])
    T_cam_tar[0:3, 3] = Pz
    T_O_tar = convert2base(T_cam_tar)
  else:
    T_O_tar = float('nan')*np.ones([4, 4])
  return T_O_tar


# ========== parameters ==========
# hand-eye calibration for Clarius ee
T_ee_cam = np.array([[0.0, -math.cos(math.pi/8), math.sin(math.pi/8), 0.0910],
                     [1.0, 0.0, 0.0, -0.0291],
                     [0.0, math.sin(math.pi/8), math.cos(math.pi/8), -0.2568],
                     [0.0, 0.0, 0.0, 1.0]])
# T_ee_cam = np.array([[0.0, -0.9272, 0.3746, 0.0886],
#                      [1.0, 0.0, 0.0, -0.0175],
#                      [0.0, 0.3746, 0.9272, -0.2889],
#                      [0.0, 0.0, 0.0, 1.0]])
# robot home pose (debug purpose)
T_O_ee = np.array([[0.0, -1.0, 0.0, 0.37],
                   [-1.0, 0.0, 0.0, 0.0],
                   [0.0, 0.0, -1.0, 0.3],
                   [0.0, 0.0, 0.0, 1.0]])
# initial target pose
T_O_tar = np.array([[0.0, -1.0, 0.0, 0.37],
                   [-1.0, 0.0, 0.0, 0.0],
                   [0.0, 0.0, -1.0, 0.3],
                   [0.0, 0.0, 0.0, 1.0]])
width = 640            # rs frame width
height = 480           # rs frame height
ss = 5                 # step size moving target
pix_tar = [320, 240]   # default target in pixel
doScan = False         # set to True to enable scanning with active-sensing ee
# ================================


rospy.init_node('scan_with_active_sensing_ee', anonymous=True)
motion = FrankaMotion(depth_compensation=True)
rs = GetRealSenseData()
cv2.namedWindow('realsense', cv2.WINDOW_AUTOSIZE)
rate = rospy.Rate(800)

while not rospy.is_shutdown():
  T_O_ee = motion.T_O_ee
  key = cv2.waitKey(3)
  rs.stream_depth2color_aligned()
  frame2show = rs.color_image
  if key & 0xFF == ord('q') or key == 27:
    cv2.destroyAllWindows()
    break

  # select landing target in camera view
  elif key == ord('w'):
    pix_tar[1] = pix_tar[1] - ss if pix_tar[1]-(ss+15) > 0 else pix_tar[1]
  elif key == ord('a'):
    pix_tar[0] = pix_tar[0] - ss if pix_tar[0]-(ss+15) > 0 else pix_tar[0]
  elif key == ord('s'):
    pix_tar[1] = pix_tar[1] + ss if pix_tar[1]+(ss+15) < height else pix_tar[1]
  elif key == ord('d'):
    pix_tar[0] = pix_tar[0] + ss if pix_tar[0]+(ss+15) < width else pix_tar[0]

  # landing pose
  elif key == ord('p'):
    print('publish target @({:d}, {:d})'.format(pix_tar[0], pix_tar[1]))
    norm, pos_tar = rs.get_surface_normal(pix_tar)
    print('target position', pos_tar, 'normal vector:', norm)
    T_O_tar = get_pose(pos_tar, norm)
    print('T_O_tar \n', T_O_tar)
  elif key == ord('g'):
    print('go to landing pose')
    motion.move2pose(T_O_tar)
    doScan = True
  elif key == ord('h'):
    print('go home')
    doScan = False
    motion.move2pose()

  # if doScan:
  #   motion.scan_w_active_sensing_ee()
  #   doScan = False

  frame2show = cv2.rectangle(
      frame2show, (pix_tar[0]-10, pix_tar[1]-10), (pix_tar[0]+10, pix_tar[1]+10), (30, 90, 30), 2, -1)
  cv2.imshow('realsense', frame2show)
  rate.sleep()
