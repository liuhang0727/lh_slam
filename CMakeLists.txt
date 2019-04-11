project(lh_slam)

cmake_minimum_required(VERSION 2.8.3)
add_compile_options(-std=c++11)
set(CMAKE_CXX_STANDARD 11)
set(CMAKE_BUILD_TYPE Release)

find_package(catkin REQUIRED COMPONENTS
  cv_bridge
  eigen_conversions
  geometry_msgs
  image_transport
  nav_msgs
  roscpp
  rospy
  sensor_msgs
  std_msgs
  tf
  tf_conversions
)

find_package(Eigen3 REQUIRED)
find_package(PCL REQUIRED)
find_package(yaml-cpp REQUIRED)

#for OpenCV2 made by myself
# set(CUDA_USE_STATIC_CUDA_RUNTIME OFF)
# set(OpenCV_DIR /usr/local/share/OpenCV)
# find_package(OpenCV REQUIRED)
# message(FATAL_ERROR ${OpenCV_DIR})

#for OpenCV3 in ROS
find_package(OpenCV REQUIRED)
# message(FATAL_ERROR ${OpenCV_DIR})

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES lh_slam
  CATKIN_DEPENDS cv_bridge eigen_conversions geometry_msgs image_transport nav_msgs roscpp rospy sensor_msgs std_msgs tf tf_conversions
  DEPENDS system_lib EIGEN3 PCL OpenCV
)

include_directories(
  /user/local/include/
  ${PROJECT_SOURCE_DIR}/include/
  ${catkin_INCLUDE_DIRS}
  ${PCL_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
)

add_subdirectory(src)
add_subdirectory(node)
