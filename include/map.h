// 
// Created by liuhang on 20190528.
// 

#ifndef MAP_H
#define MAP_H

#include "common/common_headers.h"
#include "common/cloud_utils.h"
#include "common/ros_utils.h"
#include "common/transform_utils.h"
#include "plane.h"

class map
{
    public:
    map();
    ~map();
    
    // set the params, subscribe and publish msgs
    bool init(ros::NodeHandle &node, ros::NodeHandle &private_node);


    private:

};

#endif // MAP_H