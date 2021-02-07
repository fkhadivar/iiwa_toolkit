cmake_minimum_required(VERSION 3.0.2)
project(iiwa_toolkit)

add_compile_options(-std=c++14)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  sensor_msgs
  message_generation
  passive_ds_control
  utils
  iiwa_tools
)
find_package(Eigen3 REQUIRED eigen3) 

catkin_package(
 INCLUDE_DIRS include
 LIBRARIES ${PROJECT_NAME}
 CATKIN_DEPENDS roscpp rospy std_msgs sensor_msgs message_generation passive_ds_control utils iiwa_tools
#  DEPENDS system_lib
)


include_directories( include ${catkin_INCLUDE_DIRS})
include_directories(${EIGEN3_INCLUDE_DIR})


add_executable(limbo_interface src/node_limbo_interface.cpp ${MODEL_SOURCES})
target_link_libraries(limbo_interface ${catkin_LIBRARIES} ${Eigen3_LIBRARIES})

add_executable(passive_track src/node_passive_track.cpp  src/passive_control.cpp ${MODEL_SOURCES})
target_link_libraries(passive_track ${catkin_LIBRARIES} ${Eigen3_LIBRARIES})
