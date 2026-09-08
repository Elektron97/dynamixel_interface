cmake_minimum_required(VERSION 3.0.2)
project(dynamixel_interface)

find_package(catkin REQUIRED COMPONENTS
  dynamixel_sdk
  geometry_msgs
  roscpp
  sensor_msgs
  std_msgs
)

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES ${PROJECT_NAME}
  CATKIN_DEPENDS roscpp
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

#### Libraries ####
# Register-level constants & conversion utilities, shared by every mode
add_library(dynamixel_utils src/dynamixel_utils.cpp)
target_link_libraries(dynamixel_utils ${catkin_LIBRARIES})

# Unified Dynamixel interface: command mode (current/turns) is chosen at
# construction; feedback (turns + current/torque) is always available.
add_library(${PROJECT_NAME} src/dynamixel_interface.cpp)
target_link_libraries(${PROJECT_NAME} ${catkin_LIBRARIES} dynamixel_utils)

# ROS node glue
add_library(dynamixel_node src/dynamixel_node.cpp)
target_link_libraries(dynamixel_node ${catkin_LIBRARIES} ${PROJECT_NAME})

#### Execs ####
# Hardware IO Node
add_executable(motor_io src/motor_io.cpp)
target_link_libraries(motor_io ${catkin_LIBRARIES} dynamixel_node)

#### Export Library ####
install(
  TARGETS ${PROJECT_NAME}
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_GLOBAL_BIN_DESTINATION}
)

install(
  DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
)

install(
  DIRECTORY launch/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/launch
)

catkin_install_python(
  PROGRAMS scripts/keyboard_teleop.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
