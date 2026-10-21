cmake_minimum_required(VERSION 3.8)
project(tb3_bringup)

find_package(ament_cmake REQUIRED)

# Data-only package: no targets, no source. Every directory listed here must
# exist in the source tree — CMake's install(DIRECTORY) fails the build when a
# listed directory is missing, and git does not track empty directories.
#
#   launch  every launch file in the course
#   config  semantic_targets.yaml, the canonical semantic object registry
#   worlds  the Gazebo .world files
#   models  vendored Gazebo models the .world files reference via model:// URIs
#   rviz    the RViz config nav.launch.py opens
install(DIRECTORY
  launch
  config
  worlds
  models
  rviz
  DESTINATION share/${PROJECT_NAME})

ament_package()
