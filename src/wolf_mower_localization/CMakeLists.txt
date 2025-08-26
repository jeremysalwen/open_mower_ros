cmake_minimum_required(VERSION 2.8.3)
project(wolf_mower_localization)

## Compile as C++14
add_compile_options(-std=c++14) 
# -fPIC and -rdynamic ensure unique singleton instance across shared libraries (for factories) see: https://stackoverflow.com/a/8626922
SET(CMAKE_CXX_FLAGS "-fPIC -rdynamic")

find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
  sensor_msgs
  xbot_msgs
  wolf_ros_node
)

## System dependencies are found with CMake's conventions
find_package(wolfcore REQUIRED)
find_package(wolfgnss REQUIRED)
find_package(gnss_utils REQUIRED)

###################################
## catkin specific configuration ##
###################################
catkin_package(
)

###########
## Build ##
###########

## Specify additional locations of header files
include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

add_library(subscriber_${PROJECT_NAME}
            src/subscriber_xbot_gps.cpp)

## Specify libraries to link a library or executable target against
target_link_libraries(subscriber_${PROJECT_NAME}
                      wolfcore
                      wolfgnss
                      ${catkin_LIBRARIES}
                      )

## Install libraries
install(TARGETS subscriber_${PROJECT_NAME}
   ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
   LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
   RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
