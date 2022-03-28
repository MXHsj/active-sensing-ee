#! /usr/bin/env python3
'''
obtain 2D square patch 
solve 3D normal vector from 2D square patch
'''
import numpy as np
import math

# params
edge_length = 16


def get_patch(center):
  # square patch with 9 control points
  col_vec = [int(center[0]),
             int(center[0]-edge_length/2),
             int(center[0]-edge_length/2),
             int(center[0]-edge_length/2),
             int(center[0]),
             int(center[0]+edge_length/2),
             int(center[0]+edge_length/2),
             int(center[0]+edge_length/2),
             int(center[0])]
  row_vec = [int(center[1]),
             int(center[1]+edge_length/2),
             int(center[1]),
             int(center[1]-edge_length/2),
             int(center[1]-edge_length/2),
             int(center[1]-edge_length/2),
             int(center[1]),
             int(center[1]+edge_length/2),
             int(center[1]+edge_length/2)]
  patch = np.array([col_vec, row_vec]).transpose()
  return patch


def get_normal_vector(p0, p1, p2):
  p1p0 = np.subtract(p1, p0)
  p2p0 = np.subtract(p2, p0)
  direction = np.cross(p1p0, p2p0)
  if direction[2] < 0:
    direction[0] = - direction[0]
    direction[1] = - direction[1]
    direction[2] = - direction[2]
  # magnitude = np.linalg.norm(direction)
  return direction


def get_surface_normal(point_x, point_y, point_z):
  # find normal vector of the plane P1P2P3P4
  P0 = [point_x[0], point_y[0], point_z[0]]
  P1 = [point_x[1], point_y[1], point_z[1]]
  P2 = [point_x[2], point_y[2], point_z[2]]
  P3 = [point_x[3], point_y[3], point_z[3]]
  P4 = [point_x[4], point_y[4], point_z[4]]
  P5 = [point_x[5], point_y[5], point_z[5]]
  P6 = [point_x[6], point_y[6], point_z[6]]
  P7 = [point_x[7], point_y[7], point_z[7]]
  P8 = [point_x[8], point_y[8], point_z[8]]
  # print(" P0: \n", P0, "\n P1: \n", P1, "\n P2: \n", P2, "\n P3: \n", P3, "\n P4: \n", P4)
  norm1 = get_normal_vector(P0, P1, P2)
  norm2 = get_normal_vector(P0, P2, P3)
  norm3 = get_normal_vector(P0, P3, P4)
  norm4 = get_normal_vector(P0, P4, P5)
  norm5 = get_normal_vector(P0, P5, P6)
  norm6 = get_normal_vector(P0, P6, P7)
  norm7 = get_normal_vector(P0, P7, P8)
  norm8 = get_normal_vector(P0, P1, P8)
  # averaging + normalization
  norm_vec = norm1+norm2+norm3+norm4+norm5+norm6+norm7+norm8
  return norm_vec/np.linalg.norm(norm_vec)
