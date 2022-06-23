cmake_minimum_required(VERSION 3.0.2)
#set(NODE_NAME hardware_interface)
project(stingray_hw_control)
add_compile_options(-std=c++17)
find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
  ros_control_boilerplate
  controller_manager
)
 find_package(Boost REQUIRED COMPONENTS system)
#must include empty catkin_package call
catkin_package()
include_directories(${catkin_INCLUDE_DIRS})
set(SOURCES src/stingray_hw_interface.cpp src/wave.cpp src/stingray_hw_control.cpp src/robot_fin_state.cpp)
add_executable(${PROJECT_NAME}_node ${SOURCES})

add_dependencies(${PROJECT_NAME}_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(${PROJECT_NAME}_node ${catkin_LIBRARIES})

install(
TARGETS stingray_hw_control_node
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_GLOBAL_BIN_DESTINATION}
)

#install(DIRECTORY config/ DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/config)
#install(DIRECTORY data/ DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/data)
#install(DIRECTORY launch/ DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/launch)
