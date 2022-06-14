cmake_minimum_required(VERSION 3.0.2)
project(stingray_hw_control)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
  ros_control_boilerplate
)
add_compile_options(-std=c++11)
catkin_package(INCLUDE_DIRS include)
include_directories(${catkin_INCLUDE_DIRS})
set(SOURCES src/stingray_hw_interface.cpp)
set(SOURCES src/wave.cpp)
set(SOURCES src/main.cpp)
add_executable(stingray_hw_control_node ${SOURCES})
target_link_libraries(stingray_hw_control_node ${catkin_LIBRARIES})
install(
TARGETS stingray_hw_control_node
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_GLOBAL_BIN_DESTINATION}
)

