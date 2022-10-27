cmake_minimum_required(VERSION 2.8.3)
project(osi3_bridge)

#set(CMAKE_CXX_STANDARD 11)
#set(CMAKE_CXX_STANDARD_REQUIRED ON)


find_package(catkin REQUIRED COMPONENTS
  geometry_msgs
  roscpp
  std_msgs
  message_generation
  #roslaunch
)

add_message_files(
  FILES
  Dimension3d.msg
  Orientation3d.msg
  MovingObject.msg
  TrafficUpdateMovingObject.msg
)

generate_messages(
  DEPENDENCIES
  geometry_msgs
  std_msgs
)

catkin_package(
#  INCLUDE_DIRS include
  CATKIN_DEPENDS message_runtime
  CATKIN_DEPENDS geometry_msgs roscpp std_msgs
#  DEPENDS system_lib
)

include_directories(include ${catkin_INCLUDE_DIRS})

#add_executable(${PROJECT_NAME}_publisher src/osi3_publisher.cpp 
#                                        ${NETWORK_C_FILES}) 

#add_dependencies(${PROJECT_NAME}_publisher
#  ${${PROJECT_NAME}_EXPORTED_TARGETS}
#  ${catkin_EXPORTED_TARGETS}
#)

## Specify libraries to link a library or executable target against
#target_link_libraries(${PROJECT_NAME}_publisher
#  open_simulation_interface
#  ${catkin_LIBRARIES}
#  ${Protobuf_LIBRARIES}
#)

#roslaunch_add_file_check(launch)

