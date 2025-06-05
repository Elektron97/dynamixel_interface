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
# Dynamixel Library
add_library(abstract_dyna src/dynamixel_motors.cpp)
target_link_libraries(abstract_dyna ${catkin_LIBRARIES})

add_library(ext_pos src/extpos_dynamixel.cpp)
target_link_libraries(ext_pos ${catkin_LIBRARIES} abstract_dyna)

add_library(${PROJECT_NAME} src/ros_utils.cpp)
target_link_libraries(${PROJECT_NAME} ${catkin_LIBRARIES} ext_pos)

#### Execs ####
# Hardware IO Node
add_executable(motor_io src/motor_io.cpp)
target_link_libraries(motor_io ${catkin_LIBRARIES} ${PROJECT_NAME})

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