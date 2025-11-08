cmake_minimum_required(VERSION 3.0.2)
project(camera)

## Compile as C++11, supported in ROS Kinetic and newer
# add_compile_options(-std=c++11)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  sensor_msgs
)

catkin_package(
  CATKIN_DEPENDS roscpp rospy std_msgs sensor_msgs  
)

install(PROGRAMS
  src/line_detection.py
  src/get_points.py
  src/generate_mask_from_file.py
  src/circle_detection.py
  src/check_image.py
  src/red_detection.py
  src/camera_red_detection.py
  src/line_detection_random.py
  src/mask.py
  src/color_judge.py
  src/edge_detection.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
