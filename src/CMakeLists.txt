add_library(lh_slam_lib
    feature.cpp
    plane.cpp
    global_loc.cpp
    odom.cpp
    map.cpp
)

target_link_libraries(lh_slam_lib
    ${catkin_LIBRARIES} 
    ${PCL_LIBRARIES}
    ${OpenCV_LIBRARIES}
    ${YAML_CPP_LIBRARIES}
    ${libLAS_LIBRARIES}
)