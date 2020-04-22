cmake_minimum_required(VERSION 2.8.3)
project(ubloxm8n)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  message_generation
)

add_message_files(
  FILES
  gps_gga.msg
  gps_gsa.msg
  gps_gll.msg
  gps_rmc.msg
)

generate_messages(
  DEPENDENCIES
  std_msgs
)

catkin_package(
)

include_directories(
  ${catkin_INCLUDE_DIRS}
)
