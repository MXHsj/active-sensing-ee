#! /usr/bin/env python3
import numpy as np
from cv2 import cv2
from GetSurfaceNorm import *
from pyrealsense2 import pyrealsense2 as rs


class GetRealSenseData():
  __pipeline = rs.pipeline()
  __config = rs.config()

  def __init__(self):
    self.__config.enable_stream(rs.stream.depth, 640, 480, rs.format.z16, 30)
    self.__config.enable_stream(rs.stream.color, 640, 480, rs.format.bgr8, 30)
    # process
    self.__align_depth2color = rs.align(rs.stream.color)
    self.__pc = rs.pointcloud()   # get pointcloud
    # frame data
    self.depth_image = None
    self.depth_colormap = None
    self.depth_frame = None
    self.color_image = None
    self.color_frame = None
    self.verts = None           # xyz
    self.texcoords = None       # u,v
    # start streaming
    self.__pipeline.start(self.__config)
    # camera intrinsic
    self.__profile = self.__pipeline.get_active_profile()
    self.intr = self.__profile.get_stream(rs.stream.color).as_video_stream_profile().get_intrinsics()

  def __del__(self):
    self.__pipeline.stop()

  # ====================== interface ======================
  def stream_color_frame(self):
    frames = self.__pipeline.wait_for_frames()
    self.color_frame = frames.get_color_frame()
    if self.color_frame:
      # Convert images to numpy arrays
      self.color_image = np.asanyarray(self.color_frame.get_data())

  def stream_depth_frame(self):
    frames = self.__pipeline.wait_for_frames()
    self.depth_frame = frames.get_depth_frame()
    # apply depth filters
    self.depth_filter()
    if self.depth_frame:
      # Convert images to numpy arrays
      self.depth_image = np.asanyarray(self.depth_frame.get_data())
      self.depth_colormap = cv2.applyColorMap(
          cv2.convertScaleAbs(self.depth_image, alpha=0.03), cv2.COLORMAP_JET)

  def stream_depth2color_aligned(self):
    frames = self.__pipeline.wait_for_frames()
    # align depth to color frame
    aligned_frames = self.__align_depth2color.process(frames)
    self.depth_frame = aligned_frames.get_depth_frame()
    self.color_frame = aligned_frames.get_color_frame()
    # apply depth filters
    self.depth_filter()
    if self.depth_frame and self.color_frame:
      self.depth_image = np.asanyarray(self.depth_frame.get_data())
      self.depth_colormap = cv2.applyColorMap(
          cv2.convertScaleAbs(self.depth_image, alpha=0.03), cv2.COLORMAP_JET)
      self.color_image = np.asanyarray(self.color_frame.get_data())

  def stream_pointcloud(self):
    self.stream_depth2color_aligned()
    if self.depth_frame:
      points = self.__pc.calculate(self.depth_frame)
      self.__pc.map_to(self.depth_frame)
      # Pointcloud data to arrays
      v, t = points.get_vertices(), points.get_texture_coordinates()
      self.verts = np.asanyarray(v).view(np.float32).reshape(-1, 3)   # xyz
      self.texcoords = np.asanyarray(t).view(np.float32).reshape(-1, 2)  # uv

  # ====================== utility ======================
  def depth_filter(self):
    if self.depth_frame:
      self.depth_frame = rs.decimation_filter(1).process(self.depth_frame)
      self.depth_frame = rs.disparity_transform(True).process(self.depth_frame)
      self.depth_frame = rs.spatial_filter().process(self.depth_frame)
      self.depth_frame = rs.temporal_filter().process(self.depth_frame)
      self.depth_frame = rs.disparity_transform(False).process(self.depth_frame)

  def get_xyz(self, pixels: list, flatten_out=False) -> np.ndarray:
    '''
    get xyz points from pixels in depth frame
    '''
    depth_intrin = self.depth_frame.profile.as_video_stream_profile().intrinsics
    points = []
    for i in range(len(pixels)):
      pix = pixels[i]
      try:
        depth_in_met = self.depth_frame.as_depth_frame().get_distance(pix[1], pix[0])
        pnt = rs.rs2_deproject_pixel_to_point(depth_intrin, pix, depth_in_met)
      except Exception as err:
        print(err)
        pnt = [-1, -1, -1]
      points.append(pnt)
    points_formatted = np.reshape(points, [len(pixels), 3])
    if flatten_out:
      points_formatted = np.array(points).flatten()
    return points_formatted

  def get_pix(self, points: list, flatten_out=False) -> np.ndarray:
    '''
    get pixels from points
    '''
    # TODO: complete this method
    pix = []
    pix_formatted = np.reshape(pix, [len(pix), 2])
    return pix_formatted

  def get_surface_normal(self, frame, pixel: list) -> np.ndarray:
    '''
    given a pixel in 2D image, calculate normal vector
    '''
    # TODO: test this method
    patch = get_patch(pixel)
    points = self.get_xyz(self.depth_frame, patch)
    norm = get_surface_normal(points[:, 0], points[:, 1], points[:, 2])
    return norm


# test case
if __name__ == "__main__":
  get_realsense_data = GetRealSenseData()

  cv2.namedWindow('RGB-depth', cv2.WINDOW_AUTOSIZE)
  while True:
    get_realsense_data.stream_depth2color_aligned()
    color_depth_stack = np.vstack((get_realsense_data.color_image, get_realsense_data.depth_colormap))
    cv2.imshow('RGB-depth', color_depth_stack)
    key = cv2.waitKey(10)
    if key & 0xFF == ord('Q') or key == 27:
      cv2.destroyAllWindows()
      break
