cmake_minimum_required(VERSION 2.8.3)

add_compile_options(-std=c++11)

project(cam_processing_for_grad)

find_package(catkin REQUIRED COMPONENTS
  cv_bridge
  image_geometry
  laser_geometry
  pcl_ros
  roscpp
  sensor_msgs
  std_msgs
  nav_msgs
  geometry_msgs
)

find_package(OpenCV REQUIRED)

catkin_package(
    INCLUDE_DIRS include
#  LIBRARIES targetless
    CATKIN_DEPENDS cv_bridge image_geometry pcl_ros roscpp sensor_msgs std_msgs geometry_msgs
#  DEPENDS system_lib
)

include_directories(include)
include_directories(
  ${catkin_INCLUDE_DIRS}
)

file(GLOB_RECURSE project_HEADERS
    "*.h"
    "*.hpp"
)

add_executable(scenario_one src/main.cpp)
target_link_libraries(scenario_one
  ${catkin_LIBRARIES}
   ${OpenCV_LIBS}
)


add_executable(scenario_two src/main2.cpp)
target_link_libraries(scenario_two
  ${catkin_LIBRARIES}
   ${OpenCV_LIBS}
)

