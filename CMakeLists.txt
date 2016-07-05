cmake_minimum_required(VERSION 2.8.3)
project(automatic_camera_robot_cal)

find_package(catkin REQUIRED COMPONENTS
  cv_bridge
  moveit_ros_planning_interface
  image_geometry
  image_transport
  roscpp
  rospy
  sensor_msgs
  std_msgs
)

include_directories(
  ${catkin_INCLUDE_DIRS}
)

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11 -Wall -fPIC")

catkin_package(
)

find_package(OpenCV REQUIRED COMPONENTS features2d nonfree)

# EIGEN
find_package(Eigen3 REQUIRED)
include_directories(${EIGEN3_INCLUDE_DIR})

find_package(Boost REQUIRED COMPONENTS system )
find_package(PCL 1.7 REQUIRED)

include_directories(/usr/local/include)
include_directories(${PCL_INCLUDE_DIRS})

link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

find_path(RGBD_UTILS_INCLUDE_DIR rgbd_utils/rgbd_subscriber.hpp)
find_library(RGBD_UTILS_LIBRARY NAMES rgbd_utils)

find_path(IMAGE_PROCESSING_INCLUDE_DIR image_processing/MotionDetection.h)
find_library(IMAGE_PROCESSING_LIBRARY NAMES image_processing)



add_executable(automatic_calibration src/automatic_calibration.cpp)
target_link_libraries(automatic_calibration ${catkin_LIBRARIES} ${OpenCV_LIBRARIES})

add_executable(image_convert src/image_convert.cpp)
target_link_libraries(image_convert ${catkin_LIBRARIES} ${OpenCV_LIBRARIES})

add_executable(template_matching src/template_matching.cpp)
target_link_libraries(template_matching ${catkin_LIBRARIES} ${OpenCV_LIBRARIES})

add_executable(shape_detection src/shape_detection.cpp)
target_link_libraries(shape_detection ${catkin_LIBRARIES} ${OpenCV_LIBRARIES})

add_executable(get_rotate_quat src/get_rotate_quat.cpp)
target_link_libraries(get_rotate_quat ${catkin_LIBRARIES})

add_executable(contour_detection src/contour_detection.cpp)
target_link_libraries(contour_detection ${catkin_LIBRARIES} ${OpenCV_LIBRARIES})

add_executable(marker_detection src/marker_detection.cpp)
target_link_libraries(marker_detection /usr/local/lib/libaruco.so aruco ${OpenCV_LIBRARIES} ${PCL_LIBRARIES} ${catkin_LIBRARIES} rgbd_utils image_processing)

add_executable(auto_cal_leni_method src/auto_cal_leni_method.cpp)
target_link_libraries(auto_cal_leni_method ${catkin_LIBRARIES} ${PCL_LIBRARIES} ${OpenCV_LIBRARIES} rgbd_utils image_processing)
