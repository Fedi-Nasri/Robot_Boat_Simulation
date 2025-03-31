cmake_minimum_required(VERSION 3.0.2)
project(asv_wave_sim_gazebo VERSION 1.0.0)

# Address CMake policy CMP0048 (set project version explicitly)
if(POLICY CMP0048)
  cmake_policy(SET CMP0048 NEW)
endif()

find_package(catkin REQUIRED)

catkin_package()

# Install Python scripts
catkin_install_python(PROGRAMS scripts/boat_controller.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)