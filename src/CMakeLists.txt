add_library(lh_slam_lib
    feature.cpp
    plane.cpp
)

target_link_libraries(lh_slam_lib
    ${catkin_LIBRARIES} 
    ${PCL_LIBRARIES}
    ${OpenCV_LIBRARIES}
)