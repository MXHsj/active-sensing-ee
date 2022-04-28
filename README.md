# Active-sensing end-effector (A-SEE)

## Description
ROS package for Franka ultrasound scanning using A-SEE

Franka controller: ```rv_panda```

## Usage

- bring up Franka robot
  ```
  roslaunch active_sensing_ee franka_active_sensing_ee.launch
  ```
- x, y-axis rotation + z-axis tranlation (force) control using A-SEE
  ```
  rosrun active_sensing_ee franka_motion.py
  ```
- x, y-axis rotation + z-axis tranlation (force) control using A-SEE + initial landing using RealSense
  ```
  rosrun active_sensing_ee scan_with_active_ee.py
  ```
- enable x, y-axis tranlation + z-axis rotation teleoperation control using joystick
  ```
  roslaunch active_sensing_ee franka_teleop_js.launch
  ```
