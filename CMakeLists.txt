cmake_minimum_required(VERSION 3.0.2)
project(kuavo_slam)

find_package(catkin REQUIRED COMPONENTS
  rospy
  std_msgs
  sensor_msgs
  geometry_msgs
  nav_msgs
  tf
  faster_lio
)

catkin_package()

install(DIRECTORY launch config maps nav_lidar PCD rviz
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

install(PROGRAMS
  scripts/global_localization.py
  scripts/transform_fusion.py
  scripts/publish_initial_pose.py
  scripts/start_mapping.sh
  scripts/nav_run.sh
  scripts/save_map.sh
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
