set(build_libs
    ${OpenCV_LIBRARIES}
    ${PCL_LIBRARIES}
    lh_slam_lib
)

# feature_node
add_executable(feature_node feature_node.cpp)
target_link_libraries(feature_node ${build_libs})

#faro_node
add_executable(faro_node faro_node.cpp)
target_link_libraries(faro_node ${build_libs})

