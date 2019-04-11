set(build_libs
    ${OpenCV_LIBRARIES}
    ${PCL_LIBRARIES}
    lh_slam_lib
)

add_executable(feature_node feature_node.cpp)
target_link_libraries(feature_node ${build_libs})

