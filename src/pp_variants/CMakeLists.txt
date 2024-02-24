cmake_minimum_required(VERSION 3.0.2)
project(racing_core)

## Compile as C++11, supported in ROS Kinetic and newer
# add_compile_options(-std=c++11)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  message_generation
  roscpp
  rospy
  std_msgs
  laser_geometry
  tf2
  tf2_ros
)

## System dependencies are found with CMake's conventions
# find_package(Boost REQUIRED COMPONENTS system)

## Generate messages in the 'msg' folder
 add_message_files(
   FILES
  scan_range.msg
)

## Generate added messages and services with any dependencies listed here
 generate_messages(
   DEPENDENCIES
   std_msgs
 )

###################################
## catkin specific configuration ##
###################################
## The catkin_package macro generates cmake config files for your package
## Declare things to be passed to dependent projects
## INCLUDE_DIRS: uncomment this if your package contains header files
## LIBRARIES: libraries you create in this project that dependent projects also need
## CATKIN_DEPENDS: catkin_packages dependent projects also need
## DEPENDS: system dependencies of this project that dependent projects also need
catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES racing_core
 CATKIN_DEPENDS message_generation roscpp rospy std_msgs
#  DEPENDS system_lib
)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(
# include
  ${catkin_INCLUDE_DIRS}
)


add_executable(a2 /home/nvidia/final_ws/src/racing_core/src/a2.cpp)
add_executable(pp /home/nvidia/final_ws/src/racing_core/src/pure_pursuit.cpp)
add_executable(pp_dyn /home/nvidia/final_ws/src/racing_core/src/pp_dyn.cpp)
add_executable(pp_obst /home/nvidia/final_ws/src/racing_core/src/pp_static_obst.cpp)
add_executable(full_tree_pp /home/nvidia/final_ws/src/racing_core/src/full_tree_pp.cpp)
add_executable(root_node /home/nvidia/final_ws/src/racing_core/src/root_node.cpp)


target_link_libraries(a2 ${catkin_LIBRARIES})
target_link_libraries(pp ${catkin_LIBRARIES})
target_link_libraries(pp_dyn ${catkin_LIBRARIES})
target_link_libraries(pp_obst ${catkin_LIBRARIES})
target_link_libraries(full_tree_pp ${catkin_LIBRARIES})
target_link_libraries(root_node ${catkin_LIBRARIES})

