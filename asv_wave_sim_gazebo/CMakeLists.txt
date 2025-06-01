cmake_minimum_required(VERSION 2.8.3)
project(asv_wave_sim_gazebo)
find_package(catkin REQUIRED COMPONENTS
  std_msgs
  message_generation
)

add_message_files(
  FILES
  WasteSection.msg
  WasteDetection.msg
)

generate_messages(
  DEPENDENCIES
  std_msgs
)

catkin_package(
  CATKIN_DEPENDS message_runtime std_msgs
)
