cmake_minimum_required(VERSION 2.8.3)
project(cv_debug_provider)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  cv_bridge
  image_transport
  roscpp
  sensor_msgs
)

find_package(OpenCV REQUIRED)


include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
)
catkin_package(
   INCLUDE_DIRS include
   LIBRARIES cv_debug_provider
   CATKIN_DEPENDS cv_bridge image_transport roscpp sensor_msgs
   #DEPENDS system_lib
)


add_library(cv_debug_provider src/cv_debug_provider.cpp)
target_link_libraries(cv_debug_provider
   ${catkin_LIBRARIES}
   ${OpenCV_LIBRARIES}
)
