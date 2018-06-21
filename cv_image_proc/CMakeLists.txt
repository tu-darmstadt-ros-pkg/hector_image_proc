cmake_minimum_required(VERSION 2.8.3)
project(cv_image_proc)

find_package(catkin REQUIRED COMPONENTS
  cv_bridge
  eigen_conversions
  geometry_msgs
  image_geometry
  roscpp
  sensor_msgs
  tf
  tf_conversions
)

find_package(OpenCV REQUIRED)

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES cv_image_warp cv_line_tools cv_pca
  CATKIN_DEPENDS cv_bridge eigen_conversions geometry_msgs image_geometry roscpp sensor_msgs tf tf_conversions
#  DEPENDS system_lib
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

add_library(cv_image_warp src/cv_image_warp.cpp)
target_link_libraries(cv_image_warp ${catkin_LIBRARIES})

add_library(cv_line_tools src/cv_line_tools.cpp)
target_link_libraries(cv_line_tools ${catkin_LIBRARIES})

add_library(cv_pca src/cv_pca.cpp)
target_link_libraries(cv_pca ${catkin_LIBRARIES})

install(TARGETS cv_image_warp cv_line_tools cv_pca
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})

install(DIRECTORY include/${PROJECT_NAME}/ DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION})
