
# feature_node
add_executable(feature_node feature_node.cpp)
target_link_libraries(feature_node lh_slam_lib)

# faro_node
add_executable(faro_node faro_node.cpp)
target_link_libraries(faro_node lh_slam_lib)

# global_loc_node
add_executable(global_loc_node global_loc_node.cpp)
target_link_libraries(global_loc_node lh_slam_lib)

# odom_node
add_executable(odom_node odom_node.cpp)
target_link_libraries(odom_node lh_slam_lib)

# map_node
add_executable(map_node map_node.cpp)
target_link_libraries(map_node lh_slam_lib)