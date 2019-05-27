
# feature_node
add_executable(feature_node feature_node.cpp)
target_link_libraries(feature_node lh_slam_lib)

# faro_node
add_executable(faro_node faro_node.cpp)
target_link_libraries(faro_node lh_slam_lib)