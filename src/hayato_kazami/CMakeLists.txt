cmake_minimum_required(VERSION 2.8.3)
project(hayato_kazami)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  message_generation
)

add_message_files(
  FILES
  hayato_to_pwm.msg
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
