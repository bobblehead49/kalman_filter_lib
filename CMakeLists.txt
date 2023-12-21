cmake_minimum_required(VERSION 3.0.2)
project(kalman_filter_lib)

find_package(catkin REQUIRED)

find_package(cmake_modules REQUIRED)

find_package(Eigen3 REQUIRED)

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES kalman_filter_lib
)

###########
## Build ##
###########

include_directories(
  include
  ${Eigen3_INCLUDE_DIRS}
)

add_library(kalman_filter_lib src/kalman_filter.cpp)
target_link_libraries(kalman_filter_lib ${Eigen3_LIBRARIES})
target_compile_options(kalman_filter_lib PUBLIC -std=c++17 -O3 -Wall)

#############
## Install ##
#############

install(TARGETS kalman_filter_lib
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_GLOBAL_BIN_DESTINATION}
)

install(DIRECTORY include/
  DESTINATION ${CATKIN_GLOBAL_INCLUDE_DESTINATION}
  PATTERN ".svn" EXCLUDE
)
