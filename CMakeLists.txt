cmake_minimum_required(VERSION 3.5)
project(rgbd_ros_to_lcm)

set(CMAKE_CXX_STANDARD 11)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)
SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wall -Wextra")

#find_package(catkin REQUIRED COMPONENTS
#	roscpp
#	cv_bridge
#	message_filters
#	sensor_msgs
#	image_geometry
#  pcl_ros
#)

find_package(lcm REQUIRED)
find_package(PCL REQUIRED)
find_package(OpenCV REQUIRED)
find_package(bot2-core REQUIRED CONFIG)

###################################
## catkin specific configuration ##
###################################
## The catkin_package macro generates cmake config files for your package
## Declare things to be passed to dependent projects
## INCLUDE_DIRS: uncomment this if you package contains header files
## LIBRARIES: libraries you create in this project that dependent projects also need
## CATKIN_DEPENDS: catkin_packages dependent projects also need
## DEPENDS: system dependencies of this project that dependent projects also need
#catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES jpeg_utils
#  CATKIN_DEPENDS roscpp
#)

###########
## Build ##
###########

# include_directories(include)
include_directories( 
    ${CMAKE_SOURCE_DIR}/${PROJECT_NAME}
    ${PCL_INCLUDE_DIRS} 
#    ${catkin_INCLUDE_DIRS} 
    include
)
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})


#add_executable(lcm_republisher
#  src/lcm_republisher.cpp
#)
#add_dependencies(lcm_republisher
#  ${catkin_EXPORTED_TARGETS}
#)
#target_link_libraries(lcm_republisher
#  lcm
#  ${PCL_LIBRARIES}
#  ${catkin_LIBRARIES}
#  jpeg_utils
#)

add_executable(image_extractor
  src/image_extractor.cpp
)
#add_dependencies(image_extractor
#  ${catkin_EXPORTED_TARGETS}
#)
target_link_libraries(image_extractor
  lcm
  ${OpenCV_LIBS}
  ${Boost_LIBRARIES}
  lcmtypes_bot2-core
  jpeg_utils
)

## Declare a C++ library
add_library(jpeg_utils
  src/jpeg_utils.cpp
)
#add_dependencies(jpeg_utils
##  ${catkin_EXPORTED_TARGETS}
#)
target_link_libraries(jpeg_utils
  jpeg
  z
  #${catkin_LIBRARIES}
)