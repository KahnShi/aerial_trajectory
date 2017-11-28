cmake_minimum_required(VERSION 2.8.3)
project(bspline_generator)

## Compile as C++11, supported in ROS Kinetic and newer
# add_compile_options(-std=c++11)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
set(PACKAGE_DEPENDENCIES
  roscpp
  rospy
  std_msgs
  nav_msgs
  sensor_msgs
  bspline_ros
  tf
  gap_passing
  hydrus
  aerial_robot_base
)

find_package(catkin REQUIRED COMPONENTS ${PACKAGE_DEPENDENCIES})

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES bspline_generator
  CATKIN_DEPENDS ${PACKAGE_DEPENDENCIES}
  #DEPENDS system_lib
)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

## Declare a C++ library
add_library(BsplineGenerator src/BsplineGenerator.cpp)
add_dependencies(BsplineGenerator ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(BsplineGenerator ${catkin_LIBRARIES})

add_library(AerialPlannar src/AerialPlannar.cpp)
add_dependencies(AerialPlannar ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(AerialPlannar ${catkin_LIBRARIES})

add_library(AerialControllerInterface src/AerialControllerInterface.cpp)
add_dependencies(AerialControllerInterface ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(AerialControllerInterface ${catkin_LIBRARIES})

add_executable(bspline_generator_node src/bspline_generator_node.cpp)
# set_target_properties(${PROJECT_NAME}_node PROPERTIES OUTPUT_NAME node PREFIX "")
# add_dependencies(${PROJECT_NAME}_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(bspline_generator_node
  ${catkin_LIBRARIES} BsplineGenerator)

add_executable(aerial_plannar_node src/aerial_plannar_node.cpp)
target_link_libraries(aerial_plannar_node
  ${catkin_LIBRARIES} AerialPlannar AerialControllerInterface BsplineGenerator)
