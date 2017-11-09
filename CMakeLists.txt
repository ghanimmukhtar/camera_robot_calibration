cmake_minimum_required(VERSION 2.8.3)
project(camera_robot_calibration)

find_package(catkin REQUIRED COMPONENTS
        geometry_msgs
        image_geometry
        image_transport
        roscpp
        rospy
        sensor_msgs
        std_msgs
        pcl_conversions
        cv_bridge
  	    tf
        rgbd_utils
  	    crustcrawler_core_msgs
  	    baxter_core_msgs
        message_filters
)

find_package(Boost REQUIRED COMPONENTS system )
find_package(OpenCV REQUIRED COMPONENTS features2d nonfree)
find_package(PCL 1.7 REQUIRED)
find_package(PkgConfig)
pkg_search_module(Eigen3 REQUIRED eigen3)

find_path(RGBD_UTILS_INCLUDE_DIR rgbd_utils/rgbd_subscriber.hpp)
find_library(RGBD_UTILS_LIBRARY NAMES rgbd_utils)

include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11 -Wall -fPIC")
#find_package(aruco)

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES camera_robot_calibration
  CATKIN_DEPENDS geometry_msgs image_geometry image_transport roscpp rospy sensor_msgs std_msgs pcl_conversions cv_bridge rgbd_utils tf crustcrawler_core_msgs baxter_core_msgs
)

## Specify additional locations of header files
## Your package locations should be listed before other locations
## include_directories(include /usr/local/include ${catkin_INCLUDE_DIRS} ${EIGEN3_INCLUDE_DIR} ${Boost_INCLUDE_DIRS} ${PCL_INCLUDE_DIR})
include_directories(include /opt/ros/indigo/include/)
include_directories(
        ${catkin_INCLUDE_DIR} 
        ${RGBD_UTILS_INCLUDE_DIR}
)

## Declare a C++ library
add_library(camera_robot_calibration
        src/camera_robot_calibration.cpp
        src/libs/helpers_methods.cpp
        )
target_link_libraries(camera_robot_calibration /usr/local/lib/libaruco.so ${catkin_LIBRARIES} )

## Mark executables and/or libraries for installation
install(TARGETS camera_robot_calibration
        ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
        LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
        RUNTIME DESTINATION ${CATKIN_GLOBAL_BIN_DESTINATION})


## Mark cpp header files for installation
install(DIRECTORY include/
        DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION})
